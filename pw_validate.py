import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TMJSON='/Users/john.pacino/openflight_sessions/trackman_06151200_7i_PW_DR.json'
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
# OF shots
sd={}; vbuf={}; rawbs={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected':
        sd[sn]=dict(ts=o['ts'][11:19],sod=sod(datetime.fromisoformat(o['ts'])),club=o.get('club'),
            imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'),lv=o.get('launch_angle_vertical'),src=o.get('launch_angle_vertical_source'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
# TM strokes (all clubs, for offset; PW for pairing)
d=json.load(open(TMJSON)); tm=[]
for sg in d['StrokeGroups']:
    cl={'7Iron':'7-iron','PitchingWedge':'pw','Driver':'driver'}.get(sg.get('Club'))
    for s in sg['Strokes']:
        m=s.get('Measurement') or {}
        tm.append(dict(sod=sod(datetime.fromisoformat(s['Time'])),club=cl,la=float(m['LaunchAngle'])))
# estimate OF-TM offset from all shots
diffs=np.array([o['sod']-x['sod'] for o in sd.values() for x in tm])
bins=np.arange(diffs.min(),diffs.max()+2,2.0); h,e=np.histogram(diffs,bins=bins); pk=e[h.argmax()]
off=float(np.median(diffs[(diffs>=pk-6)&(diffs<=pk+6)]))
# pair OF pw -> nearest TM pw
tmpw=[x for x in tm if x['club']=='pw']
work=[]
for sn,s in sd.items():
    if s['club']!='pw' or sn not in vbuf or sn not in rawbs: continue
    cands=[(abs((s['sod']-x['sod'])-off),x) for x in tmpw]
    dd,best=min(cands,key=lambda c:c[0])
    if dd>12: continue
    work.append((s,rawbs[sn],vbuf[sn],best['la']))
work.sort(key=lambda w:w[0]['ts'])
def mkf(b): return [KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
    arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),
    read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in b['frames']]
def angle(b,bs,imp,impk):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(b),maxlen=tr.max_buffer_frames)
    a=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return a.vertical_deg if a else None
print('offset=%.1fs  paired PW shots=%d'%(off,len(work)))
print('       ts  harness logged    TM   |Dlog| src')
maxd=0; errs=[]
for s,bs,b,tmla in work:
    v=angle(b,bs,s['imp'],s['impk'])
    dl=abs(v-s['lv']) if (v is not None and s['lv'] is not None) else float('nan')
    if v is not None and s['lv'] is not None: maxd=max(maxd,dl)
    if v is not None: errs.append(v-tmla)
    print('  %8s %s %6.1f %5.1f   %4s  %s'%(s['ts'],('%6.1f'%v if v is not None else '   nan'),(s['lv'] or 0),tmla,('%.1f'%dl if dl==dl else 'na'),s['src']))
e=np.abs(np.array(errs))
print('\nFAITHFULNESS: max|harness-live| = %.2f deg (0=exact)'%maxd)
print('PW launch MAE vs TM (harness): MAE %.2f  P50 %.2f  P90 %.2f  n=%d'%(e.mean(),np.percentile(e,50),np.percentile(e,90),len(e)))
