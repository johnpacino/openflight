import sys, json, base64, logging, math
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
import numpy as np
logging.disable(logging.WARNING)
import openflight.kld7.two_ray as TR
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TM=23.6
SN=None; sd=None; vb=None; rbs=None
for l in open(SESSION):
    o=json.loads(l)
    if o.get('type')=='shot_detected' and o.get('ts','')[11:19]=='12:21:09': SN=o['shot_number']; sd=o
for l in open(SESSION):
    o=json.loads(l)
    if o.get('type')=='kld7_buffer' and o.get('orientation')=='vertical' and o.get('shot_number')==SN: vb=o
    if o.get('type')=='rolling_buffer_capture' and o.get('shot_number')==SN: rbs=o.get('ball_speed_mph')
impk=sd['impact_timestamp_kld7']
def build(skip_t_ms=None):
    out=[]
    for d in vb['frames']:
        if not d.get('radc_b64'): continue
        t_ms=(d['timestamp']-impk)*1000.0
        if skip_t_ms is not None and abs(t_ms-skip_t_ms)<5.0: continue  # drop the club frame
        out.append({'timestamp':d['timestamp'],'radc':base64.b64decode(d['radc_b64'])})
    return out
# inventory frames in window
print('frame inventory (raw t vs impact_kld7):')
cap_ft=min(10.0,5.0*TR.M_TO_FT-5.0); t_cap=1000.0*cap_ft/max(rbs*TR.MPH_TO_FTS*math.cos(math.radians(TR.DRIFT_NOMINAL_LA_DEG)),1.0)
for d in vb['frames']:
    if not d.get('radc_b64'): continue
    t_ms=(d['timestamp']-impk)*1000.0
    if not (-120<=t_ms<=t_cap+120): continue
    vr=TR.radial_speed_mph(rbs,t_ms-TR.ACQ_MS/2.0,5.0,-4.0/12.0)
    if abs(TR.aliased_velocity_from_ball_speed_mph(vr))<=TR.DC_CORE_ALIASED_KMH:
        print('  raw_t=%6.1f  DC-skip'%t_ms); continue
    dm=TR._demodulate_frame(base64.b64decode(d['radc_b64']),t_ms,TR.expected_ball_bin_from_speed(vr),rbs,5.0,-4.0/12.0,12.0,5.0)
    print('  raw_t=%6.1f  valid=%-5s el=%5s range=%5s'%(t_ms,dm.valid,('%.1f'%dm.el_ball_deg if not math.isnan(dm.el_ball_deg) else '-'),('%.1f'%dm.range_ft if not math.isnan(dm.range_ft) else '-')))
def fit(frames,label):
    r=TR.estimate_two_ray(frames,impact_timestamp=impk,ball_speed_mph=rbs,mount_deg=10.5,angle_offset_deg=1.5,distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,net_distance_ft=10.0,range_m=5.0)
    la=r.launch_angle_deg
    fr=r.diagnostics.get('frames') or []
    print('  %-28s LA=%s  delta_vs_TM=%s  frames_used=%d %s  reason=%s'%(label,('%.1f'%la if la else 'nan'),('%+.1f'%(la-TM) if la else 'nan'),len(fr),[round(f['t_ms'],0) for f in fr],r.refusal_reason))
print('\nbaseline vs drop-club-frame (TM=%.1f):'%TM)
fit(build(),'baseline (with club frame)')
# the club frame is the early one ~ raw_t 14.9ms (corrected 10.3). drop near-impact frame
fit(build(skip_t_ms=14.9),'drop near club frame (~15ms)')
