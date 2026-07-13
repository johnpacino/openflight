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
D=5.0; ABOVE=-4.0/12.0       # tee horizontal dist & vertical offset from radar
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
h,e=np.histogram(diffs,bins=bn); pk=e[h.argmax()]; off=float(np.median(diffs[(diffs>=pk-6)&(diffs<=pk+6)]))
tmpw=[x for x in tm if x['club']=='pw']
def mkf(b): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in b['frames']]
work=[]
for sn,s in sd.items():
    if s['club']!='pw' or sn not in vbuf or sn not in rawbs: continue
    dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmpw),key=lambda c:c[0])
    if dd>12: continue
    work.append((s,rawbs[sn],vbuf[sn],best['la']))
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['diag']=r.diagnostics; return r
TR.estimate_two_ray=wrap
def el_true_from_range(R, LA):
    # ball on ray from tee at angle LA; solve s where |pos-radar|=R; el = atan2(y,x) rel radar
    c=math.cos(math.radians(LA)); s_=math.sin(math.radians(LA))
    b=2*(D*c+ABOVE*s_); cc=D*D+ABOVE*ABOVE-R*R
    disc=b*b-4*cc
    if disc<0: return None
    s=(-b+math.sqrt(disc))/2
    if s<=0: return None
    x=D+s*c; y=ABOVE+s*s_
    return math.degrees(math.atan2(y,x))
rows=[]
for s,bs,b,tmla in work:
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b),maxlen=tr.max_buffer_frames)
    CAP.clear(); tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=bs,impact_timestamp=s['impk'])
    for fr in CAP.get('diag',{}).get('frames',[]):
        R=fr['range_ft']; ei=fr['el_image_deg']
        if R is None: continue
        elt=el_true_from_range(R,tmla)
        if elt is None: continue
        sep=(abs(fr['el_deg']-ei) if ei is not None else float('nan'))
        rows.append(dict(ts=s['ts'],el=fr['el_deg'],elt=elt,bias=fr['el_deg']-elt,sep=sep,rho=fr['rho'],R=R,resid=fr['resid'],eli=ei))
TR.estimate_two_ray=_orig
rows=[r for r in rows if not math.isnan(r['sep'])]
print('per-frame:  el_meas  el_true   bias    sep   rho    R')
for r in sorted(rows,key=lambda r:r['sep']):
    print('  %-9s %6.1f %7.1f %+7.1f %6.1f %5.2f %5.1f'%(r['ts'],r['el'],r['elt'],r['bias'],r['sep'],r['rho'],r['R']))
b=np.array([r['bias'] for r in rows]); sp=np.array([r['sep'] for r in rows])
print('\ncorr(bias, sep)      = %+.2f'%np.corrcoef(b,sp)[0,1])
print('corr(bias, el_meas)  = %+.2f'%np.corrcoef(b,[r['el'] for r in rows])[0,1])
print('corr(bias, rho)      = %+.2f'%np.corrcoef(b,[r['rho'] for r in rows])[0,1])
# bin by sep
print('\nbias by separation bin:')
for lo,hi in [(0,2),(2,4),(4,8),(8,40)]:
    g=[r['bias'] for r in rows if lo<=r['sep']<hi]
    if g: print('  sep %2d-%2d:  n=%2d  mean bias %+.2f  std %.2f'%(lo,hi,len(g),np.mean(g),np.std(g)))
# linear fit bias ~ a + b*sep
A=np.vstack([np.ones_like(sp),sp]).T
coef,*_=np.linalg.lstsq(A,b,rcond=None)
pred=A@coef; resid=b-pred
print('\nfit bias = %+.2f %+.3f*sep   -> residual std %.2f deg (was %.2f unmodeled)'%(coef[0],coef[1],np.std(resid),np.std(b)))
