import sys, json, base64, logging, math
sys.path[:] = [p for p in sys.path if p not in ('/tmp','/private/tmp','')]
sys.path.insert(0, '/tmp/of-ttv/src')
from datetime import datetime
import numpy as np
logging.disable(logging.WARNING)
import openflight.kld7.two_ray as TR
print('constants: MAX_FIT_RESID=%s UMOD_RANGE=%s EL_RANGE_DEG=%s SINGLE_RAY_RHO=%s IMAGE_MAX_EL_DEG=%s MERGED_COMPONENT_DEG=%s'%(
    TR.MAX_FIT_RESID,TR.UMOD_RANGE,TR.EL_RANGE_DEG,TR.SINGLE_RAY_RHO,TR.IMAGE_MAX_EL_DEG,TR.MERGED_COMPONENT_DEG))
SESSION='/Users/john.pacino/openflight_sessions/session_20260615_120512_range.CORRECTED.jsonl'
sd={}; vbuf={}; rawbs={}
for l in open(SESSION):
    o=json.loads(l); t=o.get('type'); sn=o.get('shot_number')
    if t=='shot_detected': sd[sn]=dict(ts=o['ts'][11:19],impk=o.get('impact_timestamp_kld7'))
    elif t=='rolling_buffer_capture': rawbs[sn]=o.get('ball_speed_mph')
    elif t=='kld7_buffer' and o.get('orientation')=='vertical': vbuf[sn]=o
def rd(b):
    out=[]
    for d in b['frames']:
        fd={'timestamp':d['timestamp']}
        if d.get('radc_b64'): fd['radc']=base64.b64decode(d['radc_b64'])
        out.append(fd)
    return out
def gates(d):
    g1=d.resid<=TR.MAX_FIT_RESID
    g2=TR.UMOD_RANGE[0]<=d.umod<=TR.UMOD_RANGE[1]
    g3=TR.EL_RANGE_DEG[0]<=d.el_ball_deg<=TR.EL_RANGE_DEG[1]
    imgphys=(d.rho<TR.SINGLE_RAY_RHO) or (not math.isnan(d.el_image_deg) and (d.el_image_deg<=TR.IMAGE_MAX_EL_DEG or abs(d.el_ball_deg-d.el_image_deg)<=TR.MERGED_COMPONENT_DEG))
    return g1,g2,g3,imgphys
for tsw in ['12:25:21','12:18:23']:
    sn=[k for k,v in sd.items() if v['ts']==tsw and k in vbuf and k in rawbs][0]
    bs=rawbs[sn]; impk=sd[sn]['impk']; bore=12.0
    v_fts=bs*TR.MPH_TO_FTS; cap_ft=min(10.0,5.0*TR.M_TO_FT-5.0)
    t_cap=1000.0*cap_ft/max(v_fts*math.cos(math.radians(TR.DRIFT_NOMINAL_LA_DEG)),1.0)
    print('\n==== %s (bs=%.1f) — frames with a fitted el_ball, gate breakdown ===='%(tsw,bs))
    print('     t_ms valid | resid(<=%.3f) umod el_ball el_image rho | g_resid g_umod g_el g_imgphys'%TR.MAX_FIT_RESID)
    for fr in rd(vbuf[sn]):
        p=fr.get('radc'); ts=fr.get('timestamp')
        if p is None: continue
        t_ms=(float(ts)-float(impk))*1000.0
        if not (-120.0<=t_ms<=t_cap+120.0): continue
        vr=TR.radial_speed_mph(bs,t_ms-TR.ACQ_MS/2.0,5.0,-4.0/12.0)
        if abs(TR.aliased_velocity_from_ball_speed_mph(vr))<=TR.DC_CORE_ALIASED_KMH: continue
        d=TR._demodulate_frame(bytes(p),t_ms,TR.expected_ball_bin_from_speed(vr),bs,5.0,-4.0/12.0,bore,5.0)
        if math.isnan(d.el_ball_deg): continue  # only frames that produced a fit
        g1,g2,g3,g4=gates(d)
        print('   %6.1f  %-5s | %7.4f  %5.2f  %6.1f  %7.1f  %5.2f | %s %s %s %s'%(
            t_ms,d.valid,d.resid,d.umod,d.el_ball_deg,d.el_image_deg,d.rho,g1,g2,g3,g4))
