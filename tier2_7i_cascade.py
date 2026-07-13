import sys, json, base64, logging, math
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
import openflight.kld7.two_ray as TR
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TMJSON='/Users/john.pacino/openflight_sessions/trackman_06151200_7i_PW_DR.json'
CLUB='7-iron'; E_GATE=9
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
sd={}; vbuf={}; rawbs={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],sod=sod(datetime.fromisoformat(o['ts'])),club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
d=json.load(open(TMJSON)); tm=[]
for sg in d['StrokeGroups']:
    cl={'7Iron':'7-iron','PitchingWedge':'pw','Driver':'driver'}.get(sg.get('Club'))
    for s in sg['Strokes']:
        m=s.get('Measurement') or {}; tm.append(dict(sod=sod(datetime.fromisoformat(s['Time'])),club=cl,la=float(m['LaunchAngle'])))
diffs=np.array([o['sod']-x['sod'] for o in sd.values() for x in tm]); bn=np.arange(diffs.min(),diffs.max()+2,2.0)
hh,e=np.histogram(diffs,bins=bn); pk=e[hh.argmax()]; off=float(np.median(diffs[(diffs>=pk-6)&(diffs<=pk+6)]))
tmc=[x for x in tm if x['club']==CLUB]
def mkf(b): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in b['frames']]
work=[]
for sn,s in sd.items():
    if s['club']!=CLUB or sn not in vbuf or sn not in rawbs: continue
    dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmc),key=lambda c:c[0])
    if dd>12: continue
    work.append((s,rawbs[sn],vbuf[sn],best['la']))
work.sort(key=lambda w:w[0]['ts'])
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['d']=r.diagnostics; CAP['la']=r.launch_angle_deg; return r
TR.estimate_two_ray=wrap
def run(b,bs,imp,impk):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b),maxlen=tr.max_buffer_frames)
    CAP.clear(); ret=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk); dg=CAP.get('d',{})
    return dict(ret=(ret.vertical_deg if ret else None),pos=dg.get('la_position_deg'),sing=dg.get('la_single_frame_deg'),
        nval=dg.get('n_frames_valid') or 0,
        maxsep=max([abs(f['el_deg']-f['el_image_deg']) for f in dg.get('frames',[]) if f['el_image_deg'] is not None]+[0.0]),
        maxel=max([f['el_deg'] for f in dg.get('frames',[])]+[0.0]))
def est(r): return r['pos'] if r['pos'] is not None else (r['sing'] if r['sing'] is not None else r['ret'])
O_UMOD=TR.UMOD_RANGE; O_IMG=TR.IMAGE_MAX_EL_DEG
rows=[(s['ts'],tmla,run(b,bs,s['imp'],s['impk'])) for s,bs,b,tmla in work]
TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
A={s['ts']:run(b,bs,s['imp'],s['impk']) for s,bs,b,tmla in work}
TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG; TR.estimate_two_ray=_orig
def is_t1(rb): return rb['nval']>=2 and rb['maxsep']>=9 and rb['maxel']>=E_GATE and rb['pos'] is not None
recs=[]
for ts,tmla,rb in rows:
    ra=A[ts]
    if est(rb) is None and est(ra) is None:
        recs.append(dict(ts=ts,tm=tmla,tier='none',raw=None)); continue
    if is_t1(rb): recs.append(dict(ts=ts,tm=tmla,tier='T1',raw=est(rb)))
    elif is_t1(ra): recs.append(dict(ts=ts,tm=tmla,tier='T2a',raw=est(ra)))
    else: recs.append(dict(ts=ts,tm=tmla,tier='T2b',raw=est(ra)))
t2b=[r for r in recs if r['tier']=='T2b' and r['raw'] is not None]
y=np.array([r['raw']-r['tm'] for r in t2b])
for i,r in enumerate(t2b):
    others=[y[j] for j in range(len(t2b)) if j!=i]; r['corr']=r['raw']-np.mean(others)
def stats(es): es=sorted(abs(v) for v in es); n=len(es); return 'n=%2d MAE %.2f P50 %.2f P90 %.2f'%(n,np.mean(es),es[n//2],es[min(n-1,int(0.9*n))]) if n else 'n=0'
print('7-iron cascade (T1 gate maxel>=9; T2a mode-A re-check; T2b mode-A[+offset?]):\n')
print('  ts        TM   tier  raw   err  | true-class')
for r in sorted(recs,key=lambda r:r['ts']):
    if r['raw'] is None: print('  %s %4.1f  none   (no estimate)'%(r['ts'],r['tm'])); continue
    cls='genuine-low' if r['tm']<14 else 'normal'
    print('  %s %4.1f  %-4s %5.1f %+5.1f | %s'%(r['ts'],r['tm'],r['tier'],r['raw'],r['raw']-r['tm'],cls))
print('\nT2b subset, with vs without a blanket LOO offset — split by true class:')
for lab,sub in [('ALL T2b',t2b),('  suppressed (TM>=14)',[r for r in t2b if r['tm']>=14]),('  genuine-low (TM<14)',[r for r in t2b if r['tm']<14])]:
    print('  %-22s raw %s | +offset %s'%(lab,stats([r['raw']-r['tm'] for r in sub]),stats([r['corr']-r['tm'] for r in sub])))
print('\nFULL 7-iron, end-to-end:')
modeA_only=[ (r['raw']-r['tm']) for r in recs if r['raw'] is not None]
withoff=[ ((r['corr'] if r['tier']=='T2b' else r['raw'])-r['tm']) for r in recs if r['raw'] is not None]
print('  mode-A only (NO T2b offset): %s'%stats(modeA_only))
print('  mode-A + T2b offset        : %s'%stats(withoff))
print('  (baseline all-7i: MAE 2.25 P50 2.00)')
