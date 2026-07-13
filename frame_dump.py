import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7 import two_ray as TR
F='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TARGETS={'12:09:06':14.7,'12:11:30':16.2}  # bad single-frame vs good two-frame
sd={}; vbuf={}; rawbs={}
for l in open(F):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
def frames_for(sn):
    out=[]
    for d in vbuf[sn]['frames']:
        fd={'timestamp':d['timestamp']}
        if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
        out.append(fd)
    return out
for sn,s in sd.items():
    if s['ts'] not in TARGETS: continue
    TMv=TARGETS[s['ts']]
    res=TR.estimate_two_ray(frames_for(sn), impact_timestamp=s['impk'], ball_speed_mph=rawbs[sn],
        mount_deg=10.5, angle_offset_deg=1.5, distance_ft=5.0, ball_above_radar_ft=-4.0/12.0,
        net_distance_ft=10.0, range_m=5.0)
    d=res.diagnostics
    print('==== %s  (TM=%.1f)  raw_bs=%.1f ===='%(s['ts'],TMv,rawbs[sn]))
    print('  result: LA=%s conf=%s refuse=%s'%(res.launch_angle_deg, res.confidence, res.refusal_reason))
    print('  diag: n_valid=%s dc_core_skipped=%s tau_ms=%s la_single=%s la_curve=%s la_pos=%s'%(
        d.get('n_frames_valid'), d.get('n_frames_dc_core_skipped'), d.get('tau_range_ms'),
        d.get('la_single_frame_deg'), d.get('la_curve_deg'), d.get('la_position_deg')))
    fr=d.get('frames') or []
    print('  VALID frames (%d): t_ms / el_deg / el_image / rho / resid / range_ft'%len(fr))
    for f in fr:
        print('     t=%6.1f  el=%6.2f  el_img=%7s  rho=%5.2f  resid=%.4f  range=%5sft'%(
            f['t_ms'], f['el_deg'], f['el_image_deg'], f['rho'], f['resid'], f['range_ft']))
    print()
