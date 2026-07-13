import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.radc import extract_launch_angle, select_best_shot_result

SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.jsonl'
sd=None; vbuf=None
for l in open(SESSION):
    o=json.loads(l)
    if o.get('type')=='shot_detected' and o.get('ts','')[11:19]=='12:10:57': sd=o
    if o.get('type')=='kld7_buffer' and o.get('orientation')=='vertical' and o.get('shot_number')==10: vbuf=o
rs=vbuf['ball_angle']['radc_selection']
print('LIVE: estimator=%s final=%.1f raw=%.1f bs_used=%.1f frames=%s dc_blind=%s conf=%.2f'%(
    rs['estimator'], vbuf['ball_angle']['vertical_deg'], rs['raw_angle_deg'], rs['ball_speed_mph'],
    rs['selected_frame_indices'], rs['dc_blind_zone'], rs['confidence']))

# build dict frames with decoded 'radc'
frames=[]
for d in vbuf['frames']:
    fd=dict(d)
    if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
    frames.append(fd)
impk=sd.get('impact_timestamp_kld7'); impc=sd.get('impact_timestamp')

def run(bs, tag):
    res=extract_launch_angle(frames, ops243_ball_speed_mph=bs, angle_offset_deg=1.5,
        speed_tolerance_mph=10.0, impact_energy_threshold=3.0, centroid_floor_frac=0.5, spectrum_source='f1a',
        ops_bin_outlier_tol=25, ops_bin_outlier_penalty=10.0, ops_anchored_peak_min_snr=5.0,
        horizontal_angle_limit_deg=15.0, orientation='vertical', vertical_estimator='two_ray',
        shot_timestamp=impc, impact_timestamp=impk, mount_deg=10.5, distance_ft=5.0,
        ball_above_radar_ft=-4.0/12.0, range_m=5.0, vertical_flight_window_net_distance_ft=10.0)
    if not res: print('  %-22s -> NO RESULT'%tag); return
    b=select_best_shot_result(res)
    print('  %-22s -> final=%.1f estimator=%s frames=%s dc_blind=%s conf=%.2f raw=%s'%(
        tag, b.get('launch_angle_deg'), b.get('estimator'), b.get('selected_frame_indices'),
        b.get('dc_blind_zone'), b.get('confidence') or 0, b.get('raw_angle_deg')))

print('\nREPLAY at 5.0/off1.5/two_ray:')
run(122.26, 'corrected bs (122.26)')
run(120.6,  'raw bs (120.6, live)')
