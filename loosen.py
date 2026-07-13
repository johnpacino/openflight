import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
from openflight.kld7 import two_ray as TR
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
h,e=np.histogram(diffs,bins=bn); pk=e[h.argmax()]; off=float(np.median(diffs[(diffs>=pk-6)&(diffs<=pk+6)]))
def mkf(b): return [KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
    arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in b['frames']]
def pair(club):
    tmc=[x for x in tm if x['club']==club]; out=[]
    for sn,s in sd.items():
        if s['club']!=club or sn not in vbuf or sn not in rawbs: continue
        dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmc),key=lambda c:c[0])
        if dd>12: continue
        out.append((sn,s,best['la']))
    return out
def ang(sn,s):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(vbuf[sn]),maxlen=tr.max_buffer_frames)
    a=tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=rawbs[sn],impact_timestamp=s['impk'])
    return a.vertical_deg if a else None
def run(club):
    res={}
    for sn,s,tmla in pair(club):
        v=ang(sn,s); res[s['ts']]=(v,tmla)
    return res
def mae(res): 
    es=np.abs(np.array([v-t for v,t in res.values() if v is not None])); return es.mean(),np.percentile(es,50),np.percentile(es,90)
pw0=run('pw'); i70=run('7-iron')
# loosen the two marginal gates
TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
pw1=run('pw'); i71=run('7-iron')
for name,a,b in [('PW',pw0,pw1),('7-iron',i70,i71)]:
    m0=mae(a); m1=mae(b)
    print('%-7s  default: MAE %.2f P50 %.2f P90 %.2f   loosened: MAE %.2f P50 %.2f P90 %.2f'%(name,m0[0],m0[1],m0[2],m1[0],m1[1],m1[2]))
print('\nPW per-shot shots that CHANGED (default -> loosened):')
for ts in sorted(pw0):
    v0,t=pw0[ts]; v1,_=pw1[ts]
    if v0 is not None and v1 is not None and abs(v1-v0)>0.3:
        print('  %s TM=%4.1f  %4.1f(%+5.1f) -> %4.1f(%+5.1f)'%(ts,t,v0,v0-t,v1,v1-t))
