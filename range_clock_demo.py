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
D=5.0; ABOVE=-4.0/12.0; WRAP=5*3.28084; TEE=math.hypot(D,ABOVE)
CAP={'demods':[],'diag':None}; _od=TR._demodulate_frame; _oe=TR.estimate_two_ray
def wd(*a,**k):
    fr=_od(*a,**k); CAP['demods'].append(fr); return fr
def we(f,it,*a,**k):
    r=_oe(f,it,*a,**k); CAP['diag']=r.diagnostics; return r
TR._demodulate_frame=wd; TR.estimate_two_ray=we
def mkf(fr): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in fr]
def load(jf,cf,ts_target):
    rows={r['timestamp_of'][11:19]:r for r in csv.DictReader(open(DIR+'/'+cf))}
    sd={};vb={};rb={}
    for l in open(DIR+'/'+jf):
        o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
        if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
        elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
        elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
    sn=[k for k,v in sd.items() if v['ts']==ts_target][0]
    return sd[sn],vb[sn],rb[sn],float(rows[ts_target]['launch_v_tm'])
def go(jf,cf,ts,label):
    s,vbuf,bs,tm=load(jf,cf,ts)
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=D,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(vbuf['frames']),maxlen=tr.max_buffer_frames)
    CAP['demods']=[];CAP['diag']=None
    tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=bs,impact_timestamp=s['impk'])
    pts=sorted((t,r) for d in CAP['demods'] for (t,r) in d.sub_ranges if 0<t<75)
    # continuity unwrap
    w=0;prev=None;tu=[];ru=[]
    for t,r in pts:
        if prev is not None and (r+w*WRAP)<prev-8: w+=1
        tu.append(t);ru.append(r+w*WRAP);prev=ru[-1]
    tu=np.array(tu);ru=np.array(ru)
    # fit early-range line (first ~30ms) -> extrapolate to R=TEE => impact offset
    m=tu<30; A=np.vstack([tu[m],np.ones(m.sum())]).T; (slope,inter),*_=np.linalg.lstsq(A,ru[m],rcond=None)
    t_impact=(TEE-inter)/slope   # ms (relative to labeled t=0) where range = tee
    tau=CAP['diag'].get('tau_range_ms')
    print('\n%s  TM_LA %.1f  bs %.0f mph'%(label,tm,bs))
    print('  range track: %d clean post-impact pts, %.2f ft -> %.2f ft (unwrapped)'%(len(tu),ru.min(),ru.max()))
    print('  ball distance from radar IS the range: directly measured every ~1.8ms')
    print('  IMPACT TIME: extrapolate range back to tee (%.2f ft) -> t_impact = %+.1f ms (vs labeled t=0)'%(TEE,t_impact))
    print('               production range-anchored tau = %+.1f ms  (same thing, from the range track)'%(tau if tau else 0))
    print('  early-range slope = %.0f ft/s  (= ball speed along flight, from range alone)'%(slope*1000))
go('session_20260608_113404_trackman_pw.jsonl','compare_pw.csv','11:36:20','CLEAN PW')
go('session_20260608_104504_trackman_7i.jsonl','compare_7i.csv','10:47:01','FAST 7i (wrap shot)')
TR._demodulate_frame=_od; TR.estimate_two_ray=_oe
