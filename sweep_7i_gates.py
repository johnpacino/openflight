import sys, json, base64, logging, math, csv
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
import openflight.kld7.two_ray as TR
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['d']=r.diagnostics; CAP['la']=r.launch_angle_deg; return r
TR.estimate_two_ray=wrap
def mkf(frames): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in frames]
def runframes(frames,imp,impk,bs,mount,offset):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=offset,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=mount,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(frames),maxlen=tr.max_buffer_frames)
    CAP.clear(); ret=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk); dg=CAP.get('d',{})
    return dict(ret=(ret.vertical_deg if ret else None),pos=dg.get('la_position_deg'),sing=dg.get('la_single_frame_deg'),
        nval=dg.get('n_frames_valid') or 0,
        maxsep=max([abs(f['el_deg']-f['el_image_deg']) for f in dg.get('frames',[]) if f['el_image_deg'] is not None]+[0.0]),
        maxel=max([f['el_deg'] for f in dg.get('frames',[])]+[0.0]))
O_UMOD=TR.UMOD_RANGE; O_IMG=TR.IMAGE_MAX_EL_DEG
def est(r): return r['pos'] if r['pos'] is not None else (r['sing'] if r['sing'] is not None else r['ret'])
def cache(shots,mount,offset):
    out=[]
    for s in shots:
        rb=runframes(s['frames'],s['imp'],s['impk'],s['bs'],mount,offset)
        out.append((s['tm'],rb))
    TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
    A=[runframes(s['frames'],s['imp'],s['impk'],s['bs'],mount,offset) for s in shots]
    TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG
    return [(tm,rb,A[i]) for i,(tm,rb) in enumerate(out)]
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
# ---- 6/15 7i ----
S15='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TM15='/Users/john.pacino/openflight_sessions/trackman_06151200_7i_PW_DR.json'
sd={};vb={};rb={}
for l in open(S15):
    o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],sod=sod(datetime.fromisoformat(o['ts'])),club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
d=json.load(open(TM15));tms=[]
for sg in d['StrokeGroups']:
    if sg.get('Club')!='7Iron': continue
    for s in sg['Strokes']:
        m=s.get('Measurement') or {}; tms.append(dict(sod=sod(datetime.fromisoformat(s['Time'])),la=float(m['LaunchAngle'])))
df=np.array([o['sod']-x['sod'] for o in sd.values() if o['club']=='7-iron' for x in tms]); bnp=np.arange(df.min(),df.max()+2,2.0)
h,e=np.histogram(df,bins=bnp);pk=e[h.argmax()];offt=float(np.median(df[(df>=pk-6)&(df<=pk+6)]))
shots15=[]
for sn,s in sd.items():
    if s['club']!='7-iron' or sn not in vb or sn not in rb: continue
    dd,best=min(((abs((s['sod']-x['sod'])-offt),x) for x in tms),key=lambda c:c[0])
    if dd>12: continue
    shots15.append(dict(tm=best['la'],imp=s['imp'],impk=s['impk'],bs=rb[sn],frames=vb[sn]['frames']))
c15=cache(shots15,10.5,1.5)
# ---- 6/8 7i ----
DIR='/Users/john.pacino/openflight_sessions/trackman-6-8'
TM8={r['timestamp_of'][11:19]:float(r['launch_v_tm']) for r in csv.DictReader(open(DIR+'/compare_7i.csv')) if r.get('launch_v_tm') not in (None,'')}
sd={};vb={};rb={}
for l in open(DIR+'/session_20260608_104504_trackman_7i.jsonl'):
    o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
shots8=[]
for sn,s in sd.items():
    if sn not in vb or sn not in rb or s['ts'] not in TM8: continue
    shots8.append(dict(tm=TM8[s['ts']],imp=s['imp'],impk=s['impk'],bs=rb[sn],frames=vb[sn]['frames']))
c8=cache(shots8,10.0,2.5)
TR.estimate_two_ray=_orig
# ---- sweep gates ----
def classify(cache_,E,offset):
    def is_t1(rb): return rb['nval']>=2 and rb['maxsep']>=9 and rb['maxel']>=E and rb['pos'] is not None
    out=[]
    for tm,rb,ra in cache_:
        if est(rb) is None and est(ra) is None: out.append((tm,'none',None)); continue
        if is_t1(rb): out.append((tm,'T1',est(rb)))
        elif is_t1(ra): out.append((tm,'T2a',est(ra)))
        else: out.append((tm,'T2b',est(ra)+offset if offset is not None else est(ra)))
    return out
def fit_offset(cache_,E):  # 6/15 full-fit T2b mean under-read at gate E
    cl=classify(cache_,E,0.0)
    errs=[f-tm for tm,t,f in cl if t=='T2b' and f is not None]
    return -np.mean(errs) if errs else 0.0
def m(es,q): es=sorted(es); return np.percentile(es,q) if es else float('nan')
def stat(cl,tier=None):
    es=[abs(f-tm) for tm,t,f in cl if f is not None and (tier is None or t==tier)]
    return (len(es),np.mean(es),m(es,50),m(es,90)) if es else (0,0,0,0)
print('7-iron gate sweep — Tier-1 quality + full cascade, 6/15 (in-sample) vs 6/8 (held-out)\n')
print('  gate | 6/15 Tier-1 (n/MAE/P50)   6/15 full (n/MAE/P50/P90) | 6/8 Tier-1 (n/MAE/P50)   6/8 full (n/MAE/P50/P90/T1bias)')
for E in [14,12,11,10,9]:
    offE=fit_offset(c15,E)
    cl15=classify(c15,E,offE); cl8=classify(c8,E,offE)
    t1_15=stat(cl15,'T1'); f15=stat(cl15); t1_8=stat(cl8,'T1'); f8=stat(cl8)
    bias8=np.mean([f-tm for tm,t,f in cl8 if t=='T1' and f is not None]) if any(t=='T1' for _,t,_ in cl8) else float('nan')
    print('  >=%2d | %d/%.2f/%.2f   %d/%.2f/%.2f/%.2f | %d/%.2f/%.2f   %d/%.2f/%.2f/%.2f  bias%+.1f  (off=%.1f)'%(
        E,t1_15[0],t1_15[1],t1_15[2],f15[0],f15[1],f15[2],f15[3],
        t1_8[0],t1_8[1],t1_8[2],f8[0],f8[1],f8[2],f8[3],bias8,offE))
