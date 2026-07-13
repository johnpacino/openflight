import sys, json, base64, logging, math
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
import openflight.kld7.two_ray as TR
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
def sod(dt): return dt.hour*3600+dt.minute*60+dt.second+dt.microsecond/1e6
sd={}; vbuf={}; rawbs={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
def frames_dict(b):
    out=[]
    for d in b['frames']:
        fd={'timestamp':d['timestamp']}
        if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
        out.append(fd)
    return out
# replicate estimate_two_ray's frame loop with instrumentation
def inventory(sn, ball_speed):
    impk=sd[sn]['impk']; frames=frames_dict(vbuf[sn])
    mount,off,dist,bar,rngm,net=10.5,1.5,5.0,-4.0/12.0,5.0,10.0
    bore=mount+off
    v_fts=ball_speed*TR.MPH_TO_FTS
    wrap=rngm*TR.M_TO_FT-dist; cap_ft=min(net,wrap)
    t_cap=1000.0*cap_ft/max(v_fts*math.cos(math.radians(TR.DRIFT_NOMINAL_LA_DEG)),1.0)
    rows=[]
    for fr in frames:
        p=fr.get('radc'); ts=fr.get('timestamp')
        if p is None or ts is None: continue
        t_ms=(float(ts)-float(impk))*1000.0
        if not (-120.0<=t_ms<=t_cap+120.0): continue
        vr=TR.radial_speed_mph(ball_speed,t_ms-TR.ACQ_MS/2.0,dist,bar)
        al=TR.aliased_velocity_from_ball_speed_mph(vr)
        if abs(al)<=TR.DC_CORE_ALIASED_KMH:
            rows.append((t_ms,'dc_skip',None,None,None)); continue
        d=TR._demodulate_frame(bytes(p),t_ms,TR.expected_ball_bin_from_speed(vr),ball_speed,dist,bar,bore,rngm)
        rows.append((t_ms, 'VALID' if d.valid else 'invalid', len(d.sub_ranges),
                     None if math.isnan(d.range_ft) else round(d.range_ft,1),
                     None if math.isnan(d.el_ball_deg) else round(d.el_ball_deg,1)))
    return rows, t_cap
for label,ts_want in [('SINGLE-FRAME 12:25:21','12:25:21'),('TWO-FRAME 12:25:02','12:25:02')]:
    sn=[k for k,v in sd.items() if v['ts']==ts_want and k in vbuf and k in rawbs][0]
    rows,t_cap=inventory(sn, rawbs[sn])
    nv=sum(1 for r in rows if r[1]=='VALID')
    print('=== %s  (raw_bs=%.1f, flight cap=%.0fms) — %d frames in window, %d VALID ==='%(label,rawbs[sn],t_cap,len(rows),nv))
    print('     t_ms   status   #subranges  range  el_ball')
    for t_ms,st,nsub,rng,el in rows:
        print('   %6.1f   %-8s  %4s        %5s   %5s'%(t_ms,st,(nsub if nsub is not None else '-'),(rng if rng is not None else '-'),(el if el is not None else '-')))
    print()
