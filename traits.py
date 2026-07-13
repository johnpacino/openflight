import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7 import two_ray as TR
F='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TM={'12:06:42':15.4,'12:07:11':19.5,'12:07:40':19.0,'12:08:12':21.3,'12:08:36':16.5,
'12:09:06':14.7,'12:09:34':16.7,'12:10:05':17.4,'12:10:33':9.4,'12:10:57':16.2,
'12:11:30':16.2,'12:12:09':17.0,'12:12:37':15.1,'12:13:04':17.1,'12:13:35':13.1,
'12:14:02':10.0,'12:14:22':17.1,'12:14:50':14.5,'12:15:59':20.9}
sd={}; vbuf={}; rawbs={}
for l in open(F):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],club=o.get('club'),impk=o.get('impact_timestamp_kld7'),src=o.get('launch_angle_vertical_source'),lv=o.get('launch_angle_vertical'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
def frames_dict(sn):
    out=[]
    for d in vbuf[sn]['frames']:
        fd={'timestamp':d['timestamp']}
        if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
        out.append(fd)
    return out
rows=[]
for sn,s in sd.items():
    if s['club']!='7-iron' or sn not in vbuf or s['ts'] not in TM or sn not in rawbs or s['src']!='radar': continue
    res=TR.estimate_two_ray(frames_dict(sn), impact_timestamp=s['impk'], ball_speed_mph=rawbs[sn],
        mount_deg=10.5, angle_offset_deg=1.5, distance_ft=5.0, ball_above_radar_ft=-4.0/12.0, net_distance_ft=10.0, range_m=5.0)
    d=res.diagnostics; fr=d.get('frames') or []
    err=(s['lv']-TM[s['ts']])
    rows.append((s['ts'],TM[s['ts']],s['lv'],err,len(fr),fr,d))
rows.sort(key=lambda r:r[3])  # by error (most negative first)
print('  ts        TM   got   err  nfr | per-frame: t / el_ball / el_img / rho / resid / range')
for ts,tm,lv,err,nfr,fr,d in rows:
    tag='*BAD' if err<=-3.0 else ('  ' if abs(err)<2 else ' ~')
    fs='  '.join('%4.0f:%4.1f/%5.1f/%.2f/%.3f/%4.1f'%(f['t_ms'],f['el_deg'],(f['el_image_deg'] if f['el_image_deg'] is not None else 0),f['rho'],f['resid'],f['range_ft'] or 0) for f in fr)
    print('%s %s %4.1f %5.1f %+5.1f  %d | %s'%(tag,ts,tm,lv,err,nfr,fs))
