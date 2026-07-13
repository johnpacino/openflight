import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.jsonl'
TM={'12:06:42':15.4,'12:07:11':19.5,'12:07:40':19.0,'12:08:12':21.3,'12:08:36':16.5,
'12:09:06':14.7,'12:09:34':16.7,'12:10:05':17.4,'12:10:33':9.4,'12:10:57':16.2,
'12:11:30':16.2,'12:12:09':17.0,'12:12:37':15.1,'12:13:04':17.1,'12:13:35':13.1,
'12:14:02':10.0,'12:14:22':17.1,'12:14:50':14.5,'12:15:59':20.9}
sd={}; vbuf={}; rawbs={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'),lv=o.get('launch_angle_vertical'),src=o.get('launch_angle_vertical_source'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
work=[]
for sn,s in sd.items():
    if s['club']!='7-iron' or sn not in vbuf or s['ts'] not in TM or sn not in rawbs: continue
    if s['src']!='radar': continue   # radar shots only (estimated are club-table constants)
    frames=[KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
        arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),
        read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in vbuf[sn]['frames']]
    work.append((s, rawbs[sn], frames))
work.sort(key=lambda w: w[0]['ts'])
def angle(frames, bs, imp, impk, dist, off):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=off,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=dist,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(frames,maxlen=tr.max_buffer_frames)
    a=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return a.vertical_deg if a else None
def stats(errs):
    a=np.abs(np.array(errs)); return a.mean(),np.percentile(a,50),np.percentile(a,90),len(a)
# faithfulness check + live baseline on these radar shots
base=[(s, angle(fr,bs,s['imp'],s['impk'],5.0,1.5)) for s,bs,fr in work]
maxd=max(abs((v if v is not None else 0)-s['lv']) for s,v in base)
live_err=[s['lv']-TM[s['ts']] for s,v in base]
m,p50,p90,n=stats(live_err)
print('faithfulness: max |harness-live| over %d radar shots = %.2f deg (0=exact)'%(len(base),maxd))
print('LIVE baseline on these %d radar shots: MAE %.2f P50 %.2f P90 %.2f'%(n,m,p50,p90))
print('\n=== SWEEP (faithful, %d radar 7-iron shots) ===\n  config              MAE    P50    P90'%n)
for name,dist,off in [('5.0ft off1.5 base',5.0,1.5),('5.5ft off1.5',5.5,1.5),('5.0ft off0',5.0,0.0),('5.0ft off2.5',5.0,2.5),('5.0ft off3.5',5.0,3.5),('5.0ft off2.7',5.0,2.7)]:
    errs=[]
    for s,bs,fr in work:
        v=angle(fr,bs,s['imp'],s['impk'],dist,off)
        if v is not None: errs.append(v-TM[s['ts']])
    m,p50,p90,nn=stats(errs); print('  %-18s%6.2f %6.2f %6.2f'%(name,m,p50,p90))
