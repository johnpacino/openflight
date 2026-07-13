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
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],sod=sod(datetime.fromisoformat(o['ts'])),club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'),lv=o.get('launch_angle_vertical'))
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
def mkf(b): return [KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
    arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in b['frames']]
work=[]
for sn,s in sd.items():
    if s['club']!='pw' or sn not in vbuf or sn not in rawbs: continue
    dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmpw),key=lambda c:c[0])
    if dd>12: continue
    work.append((s,rawbs[sn],vbuf[sn],best['la']))
work.sort(key=lambda w:w[0]['ts'])
def angle(b,bs,imp,impk):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b),maxlen=tr.max_buffer_frames)
    a=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return a.vertical_deg if a else None
def run_all(): return {s['ts']:(angle(b,bs,s['imp'],s['impk']),s['lv'],tmla) for s,bs,b,tmla in work}
# --- capture originals ---
O_UMOD=TR.UMOD_RANGE; O_IMG=TR.IMAGE_MAX_EL_DEG; O_DEMOD=TR._demodulate_frame
base=run_all()
# --- mode A: loosen gates ---
TR.UMOD_RANGE=(0.6,1.5); TR.IMAGE_MAX_EL_DEG=5.0
modeA=run_all()
# --- mode A+B: also reject merged frames (sep<2.5) ---
def demodB(*a,**k):
    fr=O_DEMOD(*a,**k)
    if fr.valid and not math.isnan(fr.el_image_deg) and abs(fr.el_ball_deg-fr.el_image_deg)<2.5:
        fr.valid=False
    return fr
TR._demodulate_frame=demodB
modeAB=run_all()
TR.UMOD_RANGE=O_UMOD; TR.IMAGE_MAX_EL_DEG=O_IMG; TR._demodulate_frame=O_DEMOD
def mae(dct,key):
    es=[abs(v[key]-v[2]) for v in dct.values() if v[key] is not None]; return sum(es)/len(es), len(es)
print('  ts        TM    live_logged   modeA   modeA+B')
for ts in sorted(base):
    bl=base[ts][1]; tmla=base[ts][2]
    a=modeA[ts][0]; ab=modeAB[ts][0]
    f=lambda x: ('%5.1f'%x if x is not None else ' nan ')
    print('  %s %5.1f    %5.1f       %s   %s'%(ts,tmla,(bl if bl is not None else 0),f(a),f(ab)))
# MAE rows
bl_es=[abs(v[1]-v[2]) for v in base.values()]
print('\n  MAE vs TM:   live_logged %.2f   |   modeA %.2f   |   modeA+B %.2f'%(
    sum(bl_es)/len(bl_es), mae(modeA,0)[0], mae(modeAB,0)[0]))
