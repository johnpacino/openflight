import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7 import two_ray as TR
F='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TM={'12:06:42':15.4,'12:09:06':14.7,'12:09:34':16.7,'12:13:35':13.1,'12:14:50':14.5,
    '12:10:57':16.2,'12:07:11':19.5,'12:10:05':17.4,'12:11:30':16.2,'12:14:22':17.1}
BAD={'12:09:06','12:06:42','12:09:34','12:13:35','12:14:50'}
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
print('  ts        TM   LA   | per-frame  el_ball / el_image / |sep| / rho / range')
order=sorted(sd.items(), key=lambda kv:(sd[kv[0]]['ts'] not in BAD, sd[kv[0]]['ts']))
for sn,s in order:
    if s['ts'] not in TM: continue
    res=TR.estimate_two_ray(frames_for(sn), impact_timestamp=s['impk'], ball_speed_mph=rawbs[sn],
        mount_deg=10.5, angle_offset_deg=1.5, distance_ft=5.0, ball_above_radar_ft=-4.0/12.0, net_distance_ft=10.0, range_m=5.0)
    fr=res.diagnostics.get('frames') or []
    seps=[]
    parts=[]
    for f in fr:
        ei=f['el_image_deg']
        sep=abs(f['el_deg']-ei) if ei is not None else float('nan')
        if ei is not None: seps.append(sep)
        parts.append('%5.1f/%6s/%4.1f/%.2f/%4.1f'%(f['el_deg'], ('%.1f'%ei if ei is not None else 'na'), sep, f['rho'], f['range_ft'] or 0))
    tag='BAD ' if s['ts'] in BAD else 'good'
    minsep = min(seps) if seps else float('nan')
    print('%s %s %4.1f %5s | minsep=%4.1f | %s'%(tag,s['ts'],TM[s['ts']],
        ('%.1f'%res.launch_angle_deg if res.launch_angle_deg is not None else 'nan'), minsep, '   '.join(parts)))
