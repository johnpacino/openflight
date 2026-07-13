import sys, json, base64, logging, math, csv
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
import openflight.kld7.two_ray as TR
from openflight.kld7.two_ray import _fit_position_la
DIR='/Users/john.pacino/openflight_sessions/trackman-6-8'
DIST=5.0; ABOVE=-4.0/12.0; B0=12.0   # replay boresight (tilt 10.5 + offset 1.5)
CAP={}; _orig=TR.estimate_two_ray
def wrapf(frames, it, *a, **k):
    r=_orig(frames, it, *a, **k); CAP['d']=r.diagnostics; return r
TR.estimate_two_ray=wrapf
def mkf(fr): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in fr]
def run(fr,imp,impk,bs):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=DIST,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(fr),maxlen=tr.max_buffer_frames)
    CAP.clear(); tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return CAP.get('d',{})
def load(jf,cf):
    TM={r['timestamp_of'][11:19]:float(r['launch_v_tm']) for r in csv.DictReader(open(DIR+'/'+cf)) if r.get('launch_v_tm') not in (None,'')}
    sd={};vb={};rb={}
    for l in open(DIR+'/'+jf):
        o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
        if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
        elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
        elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
    return [dict(ts=s['ts'],tm=TM[s['ts']],imp=s['imp'],impk=s['impk'],bs=rb[sn],fr=vb[sn]['frames']) for sn,s in sd.items() if sn in vb and sn in rb and s['ts'] in TM]
# collect clean shots: >=2 frames, maxsep>=12, all used frames range in [4.5,15] (no wrap), resid ok
def clean_frames(shots,el_gate):
    # un-suppressed + un-wrapped: needs a high, in-band, well-separated anchor frame
    out=[]
    for s in shots:
        d=run(s['fr'],s['imp'],s['impk'],s['bs']); frs=d.get('frames',[])
        def sep(f): return abs(f['el_deg']-f['el_image_deg']) if f['el_image_deg'] is not None else 0.0
        usable=[f for f in frs if f['range_ft'] is not None and 4.5<=f['range_ft']<=15.0 and f['resid']<=0.03]
        anchor=[f for f in usable if f['el_deg']>=el_gate and sep(f)>=9]
        if len(usable)>=2 and anchor:
            out.append(dict(tm=s['tm'],ts=s['ts'],R=np.array([f['range_ft'] for f in usable]),
                EL=np.array([f['el_deg'] for f in usable]),W=np.array([1/(f['resid']+0.02) for f in usable])))
    return out
def la_at(c,bore):  # shift el by (bore-B0), refit position
    el=c['EL']+(bore-B0)
    return _fit_position_la(c['R'],el,c['W'],DIST,ABOVE)
def solve(clean,label):
    if not clean: print('  %s: no clean shots'%label); return
    bores=np.arange(11.0,14.01,0.1)
    bias=[np.mean([la_at(c,b)-c['tm'] for c in clean]) for b in bores]
    # zero crossing
    z=None
    for i in range(len(bores)-1):
        if bias[i]<=0<=bias[i+1] or bias[i]>=0>=bias[i+1]:
            z=bores[i]+(bores[i+1]-bores[i])*(0-bias[i])/(bias[i+1]-bias[i]); break
    print('  %-8s n=%2d  zero-bias boresight=%s  -> tilt=%s  (offset 1.5)'%(label,len(clean),
        ('%.2f'%z if z else 'none in range'),('%.2f'%(z-1.5) if z else '-')))
    for b in [11.5,12.0,12.5,13.0]:
        print('        bore %.1f (tilt %.1f): mean bias %+.2f'%(b,b-1.5,np.mean([la_at(c,b)-c['tm'] for c in clean])))
    return clean
PW=clean_frames(load('session_20260608_113404_trackman_pw.jsonl','compare_pw.csv'),14.0)
I7=clean_frames(load('session_20260608_104504_trackman_7i.jsonl','compare_7i.csv'),9.0)
TR.estimate_two_ray=_orig
print('Solve for 6/8 mount tilt from clean shots (offset fixed 1.5):\n')
solve(PW,'PW'); print(); solve(I7,'7-iron'); print(); solve(PW+I7,'COMBINED')
print('\n  (Apple Measure showed "10" = 10.0-10.99; 6/15 calibrated boresight was 12.0 = tilt 10.5)')
