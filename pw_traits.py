import sys, json, base64, logging
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
from openflight.kld7 import two_ray as TR
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
TMJSON='/Users/john.pacino/openflight_sessions/trackman_06151200_7i_PW_DR.json'
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
sd={}; vbuf={}; rawbs={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],sod=sod(datetime.fromisoformat(o['ts'])),club=o.get('club'),impk=o.get('impact_timestamp_kld7'),lv=o.get('launch_angle_vertical'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
d=json.load(open(TMJSON)); tm=[]
for sg in d['StrokeGroups']:
    cl={'7Iron':'7-iron','PitchingWedge':'pw','Driver':'driver'}.get(sg.get('Club'))
    for s in sg['Strokes']:
        m=s.get('Measurement') or {}; tm.append(dict(sod=sod(datetime.fromisoformat(s['Time'])),club=cl,la=float(m['LaunchAngle'])))
diffs=np.array([o['sod']-x['sod'] for o in sd.values() for x in tm]); bn=np.arange(diffs.min(),diffs.max()+2,2.0)
h,e=np.histogram(diffs,bins=bn); pk=e[h.argmax()]; off=float(np.median(diffs[(diffs>=pk-6)&(diffs<=pk+6)]))
tmpw=[x for x in tm if x['club']=='pw']
def frames_dict(b):
    out=[]
    for d in b['frames']:
        fd={'timestamp':d['timestamp']}
        if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
        out.append(fd)
    return out
rows=[]
for sn,s in sd.items():
    if s['club']!='pw' or sn not in vbuf or sn not in rawbs: continue
    dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmpw),key=lambda c:c[0])
    if dd>12: continue
    res=TR.estimate_two_ray(frames_dict(vbuf[sn]), impact_timestamp=s['impk'], ball_speed_mph=rawbs[sn],
        mount_deg=10.5, angle_offset_deg=1.5, distance_ft=5.0, ball_above_radar_ft=-4.0/12.0, net_distance_ft=10.0, range_m=5.0)
    fr=res.diagnostics.get('frames') or []
    err=s['lv']-best['la']
    far_el=max([f['el_deg'] for f in fr]) if fr else None
    ei=[f['el_image_deg'] for f in fr if f['el_image_deg'] is not None]; minsep=min(abs(f['el_deg']-f['el_image_deg']) for f in fr if f['el_image_deg'] is not None) if ei else None
    min_img=min(ei) if ei else None
    rows.append((s['ts'],best['la'],s['lv'],err,len(fr),far_el,min_img,minsep,fr))
rows.sort(key=lambda r:r[3])
print('  ts        TM   got   err nfr far_el min_img minsep | per-frame t/el/img/range')
for ts,tm_la,lv,err,nfr,fe,mi,ms,fr in rows:
    tag='*U' if err<=-4 else ('*O' if err>=3 else '  ')
    fs='  '.join('%3.0f:%4.1f/%5.1f/%4.1f'%(f['t_ms'],f['el_deg'],(f['el_image_deg'] if f['el_image_deg'] is not None else 0),f['range_ft'] or 0) for f in fr)
    print('%s %s %4.1f %5.1f %+5.1f %d  %5s  %6s %6s | %s'%(tag,ts,tm_la,lv,err,nfr,('%.1f'%fe if fe else 'na'),('%.1f'%mi if mi is not None else 'na'),('%.1f'%ms if ms is not None else 'na'),fs))
