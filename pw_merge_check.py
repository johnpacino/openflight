import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7 import two_ray as TR
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
sd={}; vbuf={}; rawbs={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
def fd(b):
    out=[]
    for d in b['frames']:
        e={'timestamp':d['timestamp']}
        if d.get('radc_b64'): e['radc']=base64.b64decode(d['radc_b64'])
        out.append(e)
    return out
def show(tsw,tm,note):
    sn=[k for k,v in sd.items() if v['ts']==tsw and k in vbuf and k in rawbs][0]
    res=TR.estimate_two_ray(fd(vbuf[sn]),impact_timestamp=sd[sn]['impk'],ball_speed_mph=rawbs[sn],mount_deg=10.5,angle_offset_deg=1.5,distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,net_distance_ft=10.0,range_m=5.0)
    fr=res.diagnostics.get('frames') or []
    parts=[]
    for f in fr:
        sep=abs(f['el_deg']-f['el_image_deg']) if f['el_image_deg'] is not None else None
        m='MERGED' if (sep is not None and sep<2.5) else 'clean'
        parts.append('t=%2.0f el=%4.1f img=%5.1f sep=%4.1f[%s]'%(f['t_ms'],f['el_deg'],(f['el_image_deg'] or 0),(sep if sep is not None else -1),m))
    elclimb=(max(f['el_deg'] for f in fr)-min(f['el_deg'] for f in fr)) if fr else 0
    print('  %s TM=%.1f got=%s  el-climb=%.1f  | %s   %s'%(tsw,tm,('%.1f'%res.launch_angle_deg if res.launch_angle_deg else 'nan'),elclimb,'  '.join(parts),note))
print('=== the 3 moderate PW under-readers (already two-frame, still -4 to -5) ===')
show('12:17:54',20.9,'')
show('12:21:09',23.6,'')
show('12:25:02',21.0,'')
print('\n=== reference: a 7-iron MERGE shot (+4 criteria) ===')
