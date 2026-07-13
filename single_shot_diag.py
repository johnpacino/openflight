import sys, json, base64, logging, math, csv
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
import openflight.kld7.two_ray as TR
DIR='/Users/john.pacino/openflight_sessions/trackman-6-8'
D=5.0; ABOVE=-4.0/12.0; WRAP=5*3.28084
TARGET='11:36:20'  # a clean Tier-1 PW (TM 24.7)
CAP={'demods':[]}; _od=TR._demodulate_frame
def wrapd(*a,**k):
    fr=_od(*a,**k); CAP['demods'].append(fr); return fr
TR._demodulate_frame=wrapd
def mkf(fr): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in fr]
rows={r['timestamp_of'][11:19]:r for r in csv.DictReader(open(DIR+'/compare_pw.csv'))}
sd={};vb={};rb={}
for l in open(DIR+'/session_20260608_113404_trackman_pw.jsonl'):
    o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
sn=[k for k,v in sd.items() if v['ts']==TARGET][0]
tm=float(rows[TARGET]['launch_v_tm'])
tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=D,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
tr._ring_buffer=deque(mkf(vb[sn]['frames']),maxlen=tr.max_buffer_frames)
CAP['demods']=[]; tr.get_angle_for_shot(shot_timestamp=sd[sn]['imp'],ball_speed_mph=rb[sn],impact_timestamp=sd[sn]['impk'])
pts=sorted((t,r) for d in CAP['demods'] for (t,r) in d.sub_ranges if 0.0<t<70.0)  # post-impact, pre-net
TR._demodulate_frame=_od
print('PW %s  TM %.1f  bs %.0f  -- raw sub-frame range track (t_ms, range_ft):'%(TARGET,tm,rb[sn]))
for t,r in pts: print('   %+6.1f  %5.2f'%(t,r))
# unwrap
w=0;prev=None;tu=[];ru=[]
for t,r in pts:
    if prev is not None and (r+w*WRAP)<prev-8: w+=1
    tu.append(t);ru.append(r+w*WRAP);prev=ru[-1]
tu=np.array(tu);ru=np.array(ru)
print('\n  residual-vs-LA curve (s-linearity fit; flat=no info, sharp min=usable):')
print('   LA   resid_ft   slope(ft/s)')
for LA in range(0,45,3):
    c=math.cos(math.radians(LA)); sn2=math.sin(math.radians(LA))
    b=D*c+ABOVE*sn2; disc=b*b-(D*D+ABOVE*ABOVE-ru*ru)
    if np.any(disc<0): print('   %2d   (no soln)'%LA); continue
    s=-b+np.sqrt(disc)
    A=np.vstack([tu,np.ones_like(tu)]).T; coef,*_=np.linalg.lstsq(A,s,rcond=None)
    resid=np.sqrt(np.mean((s-A@coef)**2))
    print('   %2d   %7.3f    %6.0f%s'%(LA,resid,coef[0]*1000,'   <-- TM' if abs(LA-tm)<1.5 else ''))
