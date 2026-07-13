import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from collections import deque
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7.tracker import KLD7Tracker
from openflight.kld7.types import KLD7Frame
from openflight.kld7 import two_ray as TR
F='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TM={'12:06:42':15.4,'12:07:11':19.5,'12:07:40':19.0,'12:08:12':21.3,'12:08:36':16.5,
'12:09:06':14.7,'12:09:34':16.7,'12:10:05':17.4,'12:10:33':9.4,'12:10:57':16.2,
'12:11:30':16.2,'12:12:09':17.0,'12:12:37':15.1,'12:13:04':17.1,'12:13:35':13.1,
'12:14:02':10.0,'12:14:22':17.1,'12:14:50':14.5,'12:15:59':20.9}
BAD={'12:09:06','12:06:42','12:09:34','12:13:35','12:14:50'}
LOW={'12:10:33','12:14:02'}  # genuinely low
sd={}; vbuf={}; rawbs={}
for l in open(F):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],club=o.get('club'),imp=o.get('impact_timestamp'),impk=o.get('impact_timestamp_kld7'),src=o.get('launch_angle_vertical_source'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
def fobj(sn):
    return [KLD7Frame(timestamp=d['timestamp'],radc=(base64.b64decode(d['radc_b64']) if d.get('radc_b64') else None),
        arrival_timestamp=d.get('arrival_timestamp'),complete_timestamp=d.get('complete_timestamp'),
        read_duration_ms=d.get('read_duration_ms'),done_frame_number=d.get('done_frame_number')) for d in vbuf[sn]['frames']]
def fdict(sn):
    out=[]
    for d in vbuf[sn]['frames']:
        fd={'timestamp':d['timestamp']}
        if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
        out.append(fd)
    return out
shots=[]
for sn,s in sd.items():
    if s['club']!='7-iron' or sn not in vbuf or s['ts'] not in TM or sn not in rawbs or s['src']!='radar': continue
    tr=KLD7Tracker(port=None,orientation='vertical',buffer_seconds=6.0,angle_offset_deg=1.5,range_m=5,speed_kmh=100,
        vertical_estimator='two_ray',mount_tilt_deg=10.5,ball_distance_ft=5.0,ball_above_radar_ft=-4.0/12.0,vertical_flight_window_net_distance_ft=10.0)
    tr._ring_buffer=deque(fobj(sn),maxlen=tr.max_buffer_frames)
    a=tr.get_angle_for_shot(shot_timestamp=s['imp'],ball_speed_mph=rawbs[sn],impact_timestamp=s['impk'])
    ang=a.vertical_deg if a else None
    res=TR.estimate_two_ray(fdict(sn), impact_timestamp=s['impk'], ball_speed_mph=rawbs[sn], mount_deg=10.5,
        angle_offset_deg=1.5, distance_ft=5.0, ball_above_radar_ft=-4.0/12.0, net_distance_ft=10.0, range_m=5.0)
    fr=res.diagnostics.get('frames') or []
    far_el=max([f['el_deg'] for f in fr]) if fr else None
    ei=[f['el_image_deg'] for f in fr if f['el_image_deg'] is not None]; min_img=min(ei) if ei else None
    shots.append((s['ts'],ang,far_el,min_img))
def mae(es): a=np.abs(np.array(es)); return a.mean(),np.percentile(a,50),np.percentile(a,90)
def run(gate, boost):
    es=[]; fixed=[]; wrecked=[]
    for ts,ang,fe,mi in shots:
        if ang is None or fe is None: continue
        g = gate(fe,mi)
        a = ang + (boost if g else 0)
        e=a-TM[ts]; es.append(e)
        if g and ts in BAD: fixed.append(ts)
        if g and ts in LOW: wrecked.append(ts)
    m,p50,p90=mae(es)
    return m,p50,p90,fixed,wrecked
gA=lambda fe,mi: fe is not None and fe<7.0
gB=lambda fe,mi: fe is not None and fe<7.0 and (mi is None or mi>-2.0)
print('  config                                  MAE   P50   P90  | bad fixed | genuine-low boosted')
m,p50,p90,fx,wr=run(lambda fe,mi:False,0); print('  base (no boost)                        %5.2f %5.2f %5.2f  | -         | -'%(m,p50,p90))
for name,g in [('Gate A: far_el<7',gA),('Gate B: far_el<7 & no floor img',gB)]:
    for b in [3,4]:
        m,p50,p90,fx,wr=run(g,b)
        print('  %-30s +%d  %5.2f %5.2f %5.2f  | %d/5       | %s'%(name,b,m,p50,p90,len(fx),(','.join(wr) or 'none')))
print()
print('  per-shot under Gate B +4 (which bad fixed / low wrecked / missed):')
for ts,ang,fe,mi in sorted(shots,key=lambda x:x[0]):
    if ang is None or fe is None: continue
    g=gB(fe,mi); a=ang+(4 if g else 0)
    note=''
    if ts in BAD: note='BAD '+('-> boosted' if g else '-> MISSED')
    if ts in LOW: note='LOW '+('-> BOOSTED(bad)' if g else '-> spared(ok)')
    print('   %s far_el=%4.1f min_img=%5s  base_err=%+4.1f  boosted_err=%+4.1f   %s'%(ts,fe,('%.1f'%mi if mi is not None else 'na'),ang-TM[ts],a-TM[ts],note))
print()
print('  === Gate A (far_el<7) +4 per-shot ===')
for ts,ang,fe,mi in sorted(shots,key=lambda x:x[0]):
    if ang is None or fe is None: continue
    g=gA(fe,mi); a=ang+(4 if g else 0)
    cat = 'CORRUPTED' if ts in BAD else ('genuine-low' if ts in LOW else 'normal')
    mark = ' <-- boosted' if g else ''
    print('   %s  TM=%4.1f  base=%4.1f(%+.1f)  ->  %4.1f(%+.1f)  [%s]%s'%(ts,TM[ts],ang,ang-TM[ts],a,a-TM[ts],cat,mark))
