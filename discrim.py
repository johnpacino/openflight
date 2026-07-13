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
    fr=res.diagnostics.get('frames') or []
    if not fr: continue
    el=[f['el_deg'] for f in fr]; ei=[f['el_image_deg'] for f in fr if f['el_image_deg'] is not None]
    far_el=max(el); min_img=min(ei) if ei else None
    err=s['lv']-TM[s['ts']]
    rows.append((s['ts'],TM[s['ts']],s['lv'],err,far_el,min_img,rawbs[sn]))
rows.sort(key=lambda r:r[3])
print('  ts        TM   got   err | far_el_ball  min_el_image  rawbs   <- BAD if err<=-3')
for ts,tm,lv,err,fe,mi,bs in rows:
    tag='*BAD' if err<=-3.0 else '    '
    print('%s %s %4.1f %5.1f %+5.1f |   %5.1f       %6s     %5.1f'%(tag,ts,tm,lv,err,fe,('%.1f'%mi if mi is not None else 'na'),bs))
print()
print('RULE TEST: flag if far_el_ball < 7.0 AND min_el_image > -2.0  (no clean floor image + shallow):')
for ts,tm,lv,err,fe,mi,bs in rows:
    flagged = fe<7.0 and (mi is None or mi>-2.0)
    if flagged or err<=-3: print('   %s  TM=%4.1f got=%4.1f far_el=%4.1f min_img=%5s  flagged=%s'%(ts,tm,lv,fe,('%.1f'%mi if mi is not None else 'na'),flagged))
