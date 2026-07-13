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
D=5.0; ABOVE=-4.0/12.0
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
def mkf(b): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in b['frames']]
def build(club):
    tmc=[x for x in tm if x['club']==club]; w=[]
    for sn,s in sd.items():
        if s['club']!=club or sn not in vbuf or sn not in rawbs: continue
        dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmc),key=lambda c:c[0])
        if dd>12: continue
        w.append((s,rawbs[sn],vbuf[sn],best['la']))
    return w
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['diag']=r.diagnostics; return r
TR.estimate_two_ray=wrap
def el_true(R, LA):
    c=math.cos(math.radians(LA)); s_=math.sin(math.radians(LA))
    b=2*(D*c+ABOVE*s_); cc=D*D+ABOVE*ABOVE-R*R; disc=b*b-4*cc
    if disc<0: return None
    s=(-b+math.sqrt(disc))/2
    return math.degrees(math.atan2(ABOVE+s*s_, D+s*c)) if s>0 else None
def collect(club):
    rows=[]
    for s,bs,b,tmla in build(club):
        tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
            vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
        tr._ring_buffer=deque(mkf(b),maxlen=tr.max_buffer_frames)
        CAP.clear(); tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=bs,impact_timestamp=s['impk'])
        for fr in CAP.get('diag',{}).get('frames',[]):
            R=fr['range_ft']; ei=fr['el_image_deg']
            if R is None or ei is None: continue
            elt=el_true(R,tmla)
            if elt is None: continue
            rows.append((abs(fr['el_deg']-ei), fr['el_deg']-elt))
    return rows
PW=collect('pw'); I7=collect('7-iron')
TR.estimate_two_ray=_orig
def fit(rows,label):
    sp=np.array([r[0] for r in rows]); bi=np.array([r[1] for r in rows])
    A=np.vstack([np.ones_like(sp),sp]).T; coef,*_=np.linalg.lstsq(A,bi,rcond=None)
    print('  %-8s n=%2d   bias = %+.2f %+.3f*sep   | mean bias %+.2f  std %.2f'%(label,len(rows),coef[0],coef[1],np.mean(bi),np.std(bi)))
    return coef
print('=== bias-vs-sep fit per club (does the suppression law generalize?) ===')
cpw=fit(PW,'PW'); ci7=fit(I7,'7-iron')
def apply(coef,rows):
    return np.sqrt(np.mean([(r[1]-(coef[0]+coef[1]*r[0]))**2 for r in rows]))
print('\n=== cross-apply: residual RMS of corrected bias ===')
print('  PW   frames: raw std %.2f | PW-fit %.2f | 7i-fit %.2f'%(np.std([r[1] for r in PW]),apply(cpw,PW),apply(ci7,PW)))
print('  7iron frames: raw std %.2f | 7i-fit %.2f | PW-fit %.2f'%(np.std([r[1] for r in I7]),apply(ci7,I7),apply(cpw,I7)))
# binned compare
print('\n=== mean bias by sep bin, per club ===')
for lo,hi in [(0,2),(2,8),(8,40)]:
    gp=[r[1] for r in PW if lo<=r[0]<hi]; gi=[r[1] for r in I7 if lo<=r[0]<hi]
    print('  sep %2d-%2d:  PW %s   |  7iron %s'%(lo,hi,
        ('%+.2f (n=%d)'%(np.mean(gp),len(gp)) if gp else 'none'),
        ('%+.2f (n=%d)'%(np.mean(gi),len(gi)) if gi else 'none')))
