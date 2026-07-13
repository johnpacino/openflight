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
tmpw=[x for x in tm if x['club']=='pw']
def mkf(b): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in b['frames']]
work=[]
for sn,s in sd.items():
    if s['club']!='pw' or sn not in vbuf or sn not in rawbs: continue
    dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmpw),key=lambda c:c[0])
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
    CAP.clear(); ret=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk); d=CAP.get('d',{})
    return dict(ret=(ret.vertical_deg if ret else None),pos=d.get('la_position_deg'),sing=d.get('la_single_frame_deg'),
        nval=d.get('n_frames_valid') or 0,
        maxsep=max([abs(f['el_deg']-f['el_image_deg']) for f in d.get('frames',[]) if f['el_image_deg'] is not None]+[0.0]),
        maxel=max([f['el_deg'] for f in d.get('frames',[])]+[0.0]))
O_UMOD=TR.UMOD_RANGE; O_IMG=TR.IMAGE_MAX_EL_DEG
def est(r): return r['pos'] if r['pos'] is not None else (r['sing'] if r['sing'] is not None else r['ret'])
rows=[]
for s,bs,b,tmla in work:
    rb=run(b,bs,s['imp'],s['impk']); rows.append((s['ts'],tmla,rb))
TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
A={s['ts']:run(b,bs,s['imp'],s['impk']) for s,bs,b,tmla in work}
TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG; TR.estimate_two_ray=_orig
PWE=14
def is_t1(rb): return rb['nval']>=2 and rb['maxsep']>=9 and rb['maxel']>=PWE and rb['pos'] is not None
# classify
recs=[]
for ts,tmla,rb in rows:
    ra=A[ts]
    if is_t1(rb): recs.append(dict(ts=ts,tm=tmla,tier='T1',raw=est(rb),maxelA=ra['maxel']))
    elif is_t1(ra): recs.append(dict(ts=ts,tm=tmla,tier='T2a',raw=est(ra),maxelA=ra['maxel']))   # mode-A recovered to clean
    else: recs.append(dict(ts=ts,tm=tmla,tier='T2b',raw=est(ra),maxelA=ra['maxel']))             # still suppressed
# LOO correction for T2b: predict err(=raw-tm) from maxelA; also constant model
t2b=[r for r in recs if r['tier']=='T2b']
x=np.array([r['maxelA'] for r in t2b]); y=np.array([r['raw']-r['tm'] for r in t2b])  # err (negative)
def loo_pred(i,model):
    m=[j for j in range(len(t2b)) if j!=i]; xx,yy=x[m],y[m]
    if model=='const': return np.mean(yy)
    A_=np.vstack([np.ones_like(xx),xx]).T; c,*_=np.linalg.lstsq(A_,yy,rcond=None); return c[0]+c[1]*x[i]
for model in ['const','linear']:
    for i,r in enumerate(t2b): r['corr_'+model]=r['raw']-loo_pred(i,model)
def stats(es): es=sorted(abs(v) for v in es); n=len(es); return 'n=%2d MAE %.2f P50 %.2f P90 %.2f'%(n,np.mean(es),es[n//2],es[min(n-1,int(0.9*n))])
print('PW cascade tiers:')
for r in recs:
    extra=''
    if r['tier']=='T2b': extra='  corr(const) %+.1f  corr(lin) %+.1f'%(r.get('corr_const')-r['tm'],r.get('corr_linear')-r['tm'])
    print('  %s  TM %4.1f  %-4s raw %5.1f (err %+5.1f) maxelA %.1f%s'%(r['ts'],r['tm'],r['tier'],r['raw'],r['raw']-r['tm'],r['maxelA'],extra))
print('\n--- T2b reject subset (needs correction) ---')
print('  raw (mode-A only)     : %s'%stats([r['raw']-r['tm'] for r in t2b]))
print('  + LOO const offset    : %s'%stats([r['corr_const']-r['tm'] for r in t2b]))
print('  + LOO linear(maxel)   : %s'%stats([r['corr_linear']-r['tm'] for r in t2b]))
print('\n--- FULL PW (all shots, cascade end-to-end, T2b uses LOO-linear) ---')
allerr=[]
for r in recs:
    v = r['corr_linear'] if r['tier']=='T2b' else r['raw']
    allerr.append(v-r['tm'])
print('  %s'%stats(allerr))
print('  (baseline all-PW for comparison: MAE 4.19 P50 4.02)')
