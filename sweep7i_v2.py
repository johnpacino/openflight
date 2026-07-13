import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
F='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'  # 12:15:59 = 7-iron
TM={'12:06:42':15.4,'12:07:11':19.5,'12:07:40':19.0,'12:08:12':21.3,'12:08:36':16.5,
'12:09:06':14.7,'12:09:34':16.7,'12:10:05':17.4,'12:10:33':9.4,'12:10:57':16.2,
'12:11:30':16.2,'12:12:09':17.0,'12:12:37':15.1,'12:13:04':17.1,'12:13:35':13.1,
'12:14:02':10.0,'12:14:22':17.1,'12:14:50':14.5,'12:15:59':20.9}
sd={}; vbuf={}; rawbs={}
for l in open(F):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'),lv=o.get('launch_angle_vertical'),src=o.get('launch_angle_vertical_source'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
radar=[]; const_errs=[]
for sn,s in sd.items():
    if s['club']!='7-iron' or s['ts'] not in TM: continue
    if s['src']=='estimated':
        const_errs.append(s['lv']-TM[s['ts']]); continue   # club-table constant, geometry-invariant
    if sn not in vbuf or sn not in rawbs: continue
    frames=[KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
        arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),
        read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in vbuf[sn]['frames']]
    radar.append((s, rawbs[sn], frames))
def angle(fr,bs,imp,impk,dist,off):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=off,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=dist,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(fr,maxlen=tr.max_buffer_frames)
    a=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk); return a.vertical_deg if a else None
def st(errs):
    a=np.abs(np.array(errs)); return a.mean(),np.percentile(a,50),np.percentile(a,90),len(a)
print('radar shots=%d  +  club-table constants=%d  = %d total'%(len(radar),len(const_errs),len(radar)+len(const_errs)))
print('\n  config            | radar-only (n=%d)      | full incl. 2 const (n=%d)'%(len(radar),len(radar)+len(const_errs)))
print('                    |  MAE   P50   P90      |  MAE   P50   P90')
for name,dist,off in [('5.0 off1.5 (base)',5.0,1.5),('5.5 off1.5',5.5,1.5),('5.0 off2.5',5.0,2.5),('5.0 off2.7',5.0,2.7),('5.0 off3.5',5.0,3.5),('5.0 off0',5.0,0.0)]:
    re=[angle(fr,bs,s['imp'],s['impk'],dist,off)-TM[s['ts']] for s,bs,fr in radar if angle(fr,bs,s['imp'],s['impk'],dist,off) is not None]
    m,p,n,_=st(re); fa=st(re+const_errs)
    print('  %-17s |%5.2f %5.2f %5.2f      |%5.2f %5.2f %5.2f'%(name,m,p,n,fa[0],fa[1],fa[2]))
