import sys, json, base64, logging, math
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
import numpy as np
logging.disable(logging.WARNING)
import openflight.kld7.two_ray as TR
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
print('ACQ_MS=%s SAMPLE_DT_MS=%s'%(TR.ACQ_MS, getattr(TR,'SAMPLE_DT_MS','?')))
SN=None; sd=None; vb=None
for l in open(SESSION):
    o=json.loads(l)
    if o.get('type')=='shot_detected' and o.get('ts','')[11:19]=='12:21:09':
        SN=o['shot_number']; sd=o
for l in open(SESSION):
    o=json.loads(l)
    if o.get('type')=='kld7_buffer' and o.get('orientation')=='vertical' and o.get('shot_number')==SN: vb=o
print('\n--- impact timestamps ---')
print('  impact_timestamp(OPS) =',sd.get('impact_timestamp'))
print('  impact_timestamp_kld7 =',sd.get('impact_timestamp_kld7'))
print('  ball_speed(corr)=%.1f club_speed=%.1f'%(sd.get('ball_speed_mph') or 0, sd.get('club_speed_mph') or 0))
ba=vb['ball_angle']['radc_selection']; ca=(vb.get('club_angle') or {}).get('radc_selection')
print('\n--- LIVE logged BALL angle ---')
print('  vertical_deg=%s raw_angle=%s selected_t_ms=%s frames=%s estimator=%s'%(vb['ball_angle']['vertical_deg'],ba.get('raw_angle_deg'),ba.get('selected_t_ms'),ba.get('selected_frame_indices'),ba.get('estimator')))
print('--- LIVE logged CLUB angle (same buffer, club speed) ---')
if ca:
    print('  vertical_deg=%s raw_angle=%s selected_t_ms=%s frames=%s ball_speed_used=%s'%(vb['club_angle']['vertical_deg'],ca.get('raw_angle_deg'),ca.get('selected_t_ms'),ca.get('selected_frame_indices'),ca.get('ball_speed_mph')))
# two_ray near-frame raw timing
def fd(b):
    out=[]
    for d in b['frames']:
        e={'timestamp':d['timestamp']}
        if d.get('radc_b64'): e['radc']=base64.b64decode(d['radc_b64'])
        out.append(e)
    return out
impk=sd.get('impact_timestamp_kld7'); rbs=None
for l in open(SESSION):
    o=json.loads(l)
    if o.get('type')=='rolling_buffer_capture' and o.get('shot_number')==SN: rbs=o.get('ball_speed_mph')
res=TR.estimate_two_ray(fd(vb),impact_timestamp=impk,ball_speed_mph=rbs,mount_deg=10.5,angle_offset_deg=1.5,distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,net_distance_ft=10.0,range_m=5.0)
tau=res.diagnostics.get('tau_range_ms')
print('\n--- two_ray frames (tau=%s ms): corrected_t = raw_t_center + tau ---'%tau)
for f in (res.diagnostics.get('frames') or []):
    print('   corrected_t=%5.1f  => raw_t_center=%5.1f  el=%4.1f img=%5.1f range=%4.1f'%(f['t_ms'],f['t_ms']-tau,f['el_deg'],(f['el_image_deg'] or 0),f['range_ft'] or 0))
