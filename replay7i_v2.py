import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.radc import extract_launch_angle, select_best_shot_result

SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.jsonl'
TM={'12:06:42':15.4,'12:07:11':19.5,'12:07:40':19.0,'12:08:12':21.3,'12:08:36':16.5,
'12:09:06':14.7,'12:09:34':16.7,'12:10:05':17.4,'12:10:33':9.4,'12:10:57':16.2,
'12:11:30':16.2,'12:12:09':17.0,'12:12:37':15.1,'12:13:04':17.1,'12:13:35':13.1,
'12:14:02':10.0,'12:14:22':17.1,'12:14:50':14.5,'12:15:59':20.9}
sd={}; vbuf={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type')
    if t=='shot_detected':
        sd[o['shot_number']]=dict(ts=o['ts'][11:19],club=o.get('club'),imp=o.get('impact_timestamp'),
            impk=o.get('impact_timestamp_kld7'),lv=o.get('launch_angle_vertical'),src=o.get('launch_angle_vertical_source'))
    elif t=='kld7_buffer' and o.get('orientation')=='vertical':
        vbuf[o['shot_number']]=o
work=[]
for sn,s in sd.items():
    if s['club']!='7-iron' or sn not in vbuf or s['ts'] not in TM: continue
    b=vbuf[sn]
    raw_bs=(b.get('ball_angle') or {}).get('radc_selection',{}).get('ball_speed_mph')
    frames=[]
    for d in b['frames']:
        fd=dict(d)
        if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
        frames.append(fd)
    work.append((s, raw_bs, frames))
work.sort(key=lambda w: w[0]['ts'])

def angle(frames, bs, imp, impk, dist, off):
    if bs is None: return None
    res=extract_launch_angle(frames, ops243_ball_speed_mph=bs, angle_offset_deg=off,
        speed_tolerance_mph=10.0, impact_energy_threshold=3.0, centroid_floor_frac=0.5, spectrum_source='f1a',
        ops_bin_outlier_tol=25, ops_bin_outlier_penalty=10.0, ops_anchored_peak_min_snr=5.0,
        horizontal_angle_limit_deg=15.0, orientation='vertical', vertical_estimator='two_ray',
        shot_timestamp=imp, impact_timestamp=impk, mount_deg=10.5, distance_ft=dist,
        ball_above_radar_ft=-4.0/12.0, range_m=5.0, vertical_flight_window_net_distance_ft=10.0)
    if not res: return None
    b=select_best_shot_result(res)
    return b.get('launch_angle_deg') if b else None

def run(dist, off):
    return [(s['ts'], angle(fr,bs,s['imp'],s['impk'],dist,off), s['lv'], TM[s['ts']], s['src']) for s,bs,fr in work]
def stats(per):
    e=np.abs(np.array([v-tm for ts,v,lg,tm,src in per if v is not None]))
    return e.mean(),np.percentile(e,50),np.percentile(e,90),len(e)

print('=== VALIDATION: harness(5.0,off1.5, RAW bs) vs LOGGED ===')
print('       ts harness logged    TM  src')
for ts,v,lg,tm,src in run(5.0,1.5):
    print('  %8s %s %6.1f %5.1f  %s'%(ts,('%6.1f'%v if v is not None else '   nan'),(lg or 0),tm,src))
m,p50,p90,n=stats(run(5.0,1.5))
print('  -> MAE %.2f P50 %.2f P90 %.2f n=%d   (live logged: 1.92/1.22/4.52, n=19)'%(m,p50,p90,n))
print('\n=== SWEEP (7-iron, RAW bs) ===\n  config              MAE    P50    P90   n')
for name,dist,off in [('5.0ft off1.5 base',5.0,1.5),('5.5ft off1.5',5.5,1.5),('5.0ft off0',5.0,0.0),('5.0ft off2.5',5.0,2.5),('5.0ft off3.5',5.0,3.5)]:
    m,p50,p90,n=stats(run(dist,off)); print('  %-18s%6.2f %6.2f %6.2f %3d'%(name,m,p50,p90,n))
