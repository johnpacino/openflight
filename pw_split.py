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
        m=s.get('Measurement') or {}; tm.append(dict(sod=sod(datetime.fromisoformat(s['Time'])),club=cl,la=float(m['LaunchAngle']),bs=float(m['BallSpeed'])*2.236936,cs=float(m.get('ClubSpeed',0))*2.236936))
diffs=np.array([o['sod']-x['sod'] for o in sd.values() for x in tm]); bn=np.arange(diffs.min(),diffs.max()+2,2.0)
h,e=np.histogram(diffs,bins=bn); pk=e[h.argmax()]; off=float(np.median(diffs[(diffs>=pk-6)&(diffs<=pk+6)]))
tmpw=[x for x in tm if x['club']=='pw']
def fdict(b):
    out=[]
    for d in b['frames']:
        fd={'timestamp':d['timestamp']}
        if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
        out.append(fd)
    return out
GATED=['12:25:21','12:18:23','12:17:28','12:17:54','12:21:09','12:19:11','12:25:02']
BIG={'12:25:21','12:18:23','12:17:28'}
for sn,s in sd.items():
    if s['ts'] not in GATED: continue
    dd,best=min(((abs((s['sod']-x['sod'])-off),x) for x in tmpw),key=lambda c:c[0])
    res=TR.estimate_two_ray(fdict(vbuf[sn]), impact_timestamp=s['impk'], ball_speed_mph=rawbs[sn], mount_deg=10.5, angle_offset_deg=1.5, distance_ft=5.0, ball_above_radar_ft=-4.0/12.0, net_distance_ft=10.0, range_m=5.0)
    dg=res.diagnostics; fr=dg.get('frames') or []
    s['_o']=(best['la'],s['lv'],best['bs'],best['cs'],rawbs[sn],res.confidence,dg,fr)
def show(ts):
    for sn,s in sd.items():
        if s['ts']==ts:
            tmla,got,tmbs,tmcs,rbs,conf,dg,fr=s['_o']
            grp='BIG ' if ts in BIG else 'small'
            print('%s %s TM=%4.1f got=%4.1f miss=%+5.1f | conf=%.2f nfr=%s dc_skip=%s tau=%s la_single=%s la_curve=%s la_pos=%s | TMbs=%.0f TMcs=%.0f'%(
                grp,ts,tmla,got,got-tmla,conf or 0,dg.get('n_frames_valid'),dg.get('n_frames_dc_core_skipped'),dg.get('tau_range_ms'),
                dg.get('la_single_frame_deg'),dg.get('la_curve_deg'),dg.get('la_position_deg'),tmbs,tmcs))
            for f in fr:
                print('       frame t=%5.1f el=%5.1f img=%6.1f rho=%.2f resid=%.4f range=%5.1f'%(f['t_ms'],f['el_deg'],(f['el_image_deg'] or 0),f['rho'],f['resid'],f['range_ft'] or 0))
print('=== BIG misses ===')
for ts in ['12:25:21','12:18:23','12:17:28']: show(ts)
print('=== small misses ===')
for ts in ['12:17:54','12:21:09','12:19:11','12:25:02']: show(ts)
