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
MOUNT=10.0; OFFSET=2.5; ABOVE=-4.0/12.0; DIST=5.0
BAD={'10:47:01','10:48:09','10:48:32'}; GOOD={'10:46:18','10:46:39'}
TARGETS=BAD|GOOD
TM={r['timestamp_of'][11:19]:float(r['launch_v_tm']) for r in csv.DictReader(open(DIR+'/compare_7i.csv')) if r.get('launch_v_tm') not in (None,'')}
sd={};vb={};rb={}
for l in open(DIR+'/session_20260608_104504_trackman_7i.jsonl'):
    o=json.loads(l);t=o.get('type');sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rb[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vb[sn]=o
def mkf(frames): return [KLD7Frame(timestamp=q['timestamp'],radc=(base64.b64decode(q['radc_b64']) if q.get('radc_b64') else None),
    arrival_timestamp=q.get('arrival_timestamp'),complete_timestamp=q.get('complete_timestamp'),read_duration_ms=q.get('read_duration_ms'),done_frame_number=q.get('done_frame_number')) for q in frames]
CAP={}; _orig=TR.estimate_two_ray
def wrap(frames, impact_timestamp, *a, **k):
    r=_orig(frames, impact_timestamp, *a, **k); CAP['d']=r.diagnostics; CAP['la']=r.launch_angle_deg; CAP['conf']=r.confidence; return r
TR.estimate_two_ray=wrap
def implied_la(R,el):
    e=math.radians(el); bx=R*math.cos(e)-DIST; by=R*math.sin(e)-ABOVE
    return math.degrees(math.atan2(by,bx)) if bx>0 else float('nan')
def run(frames,imp,impk,bs):
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=OFFSET,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=MOUNT,ball_distance_ft=DIST,ball_above_radar_ft=ABOVE,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(mkf(frames),maxlen=tr.max_buffer_frames)
    CAP.clear(); ret=tr.get_angle_for_shot(shot_timestamp=imp,ball_speed_mph=bs,impact_timestamp=impk)
    return (ret.vertical_deg if ret else None),CAP.get('d',{}),CAP.get('conf')
rows=[]
for sn,s in sd.items():
    if s['ts'] not in TARGETS or sn not in vb or sn not in rb: continue
    rows.append((s['ts'],s,rb[sn],vb[sn]['frames']))
for ts,s,bs,frames in sorted(rows,key=lambda r:r[0]):
    la,dg,conf=run(frames,s['imp'],s['impk'],bs)
    tag='BAD ' if ts in BAD else 'good'
    print('='*78)
    print('%s %s   TM %.1f   returned LA %s (conf %s)   bs %.0f'%(tag,ts,TM[ts],('%.1f'%la if la else None),conf,bs))
    print('   la_curve=%s  la_position=%s  la_single=%s  nval=%s'%(dg.get('la_curve_deg'),dg.get('la_position_deg'),dg.get('la_single_frame_deg'),dg.get('n_frames_valid')))
    frs=dg.get('frames',[])
    if frs:
        print('   frame:  t_ms   el_deg  el_img    rho   resid  range_ft | implied_LA')
        for f in frs:
            ei=f['el_image_deg']; R=f['range_ft']
            il=implied_la(R,f['el_deg']) if R is not None else float('nan')
            print('          %+6.1f  %6.2f  %6s  %5.2f  %.4f  %7s |  %s'%(
                f['t_ms'],f['el_deg'],('%.2f'%ei if ei is not None else 'nan'),f['rho'],f['resid'],
                ('%.2f'%R if R is not None else 'nan'),('%.1f'%il if not math.isnan(il) else 'nan')))
TR.estimate_two_ray=_orig
