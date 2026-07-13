import json, numpy as np
recs=json.load(open('/tmp/of-ttv/recs.json'))
THRESH={'7-iron':9,'pw':14}
def t1(r):
    E=THRESH.get(r['club']); return E is not None and r['nval']>=2 and r['maxsep']>=9 and r['maxel']>=E and r['pos'] is not None
def primary(r):
    if r['pos'] is not None: return r['pos'],'pos'
    if r['sing'] is not None: return r['sing'],'sing'
    return r['live'],'live'
# reject pile, irons+wedges only
pile=[r for r in recs if r['club'] in THRESH and not t1(r)]
for r in pile:
    p,src=primary(r); r['prim']=p; r['src']=src; r['err']=(p-r['tm']) if p is not None else None
def cat(r):
    if r['tm']<14: return 'genuine-low (TM<14, AMBIGUOUS)'
    if r['src']=='sing': return 'single-frame collapse (suppressed)'
    if r['err'] is not None and r['err']<-1.5: return '2-frame suppressed (reads low)'
    if r['err'] is not None and r['err']>1.5: return 'over-read'
    return 'rejected-but-accurate'
from collections import defaultdict
groups=defaultdict(list)
for r in pile: groups[cat(r)].append(r)
print('=== Tier-2 reject pile (irons+wedges): %d shots ==='%len(pile))
for g in ['single-frame collapse (suppressed)','2-frame suppressed (reads low)','genuine-low (TM<14, AMBIGUOUS)','over-read','rejected-but-accurate']:
    rows=groups.get(g,[])
    if not rows: continue
    es=[abs(r['err']) for r in rows if r['err'] is not None]
    print('\n  [%s]  n=%d   MAE %.2f'%(g,len(rows),np.mean(es) if es else 0))
    for r in sorted(rows,key=lambda r:(r['club'],r['ts'])):
        if r['prim'] is None:
            print('    %-7s %s  TM %4.1f  NO ESTIMATE (refused/fallback)'%(r['club'][:7],r['ts'],r['tm'])); continue
        print('    %-7s %s  TM %4.1f  %s=%4.1f  err %+5.1f | nval %d maxel %.1f maxsep %.1f bs %.0f'%(
            r['club'][:7],r['ts'],r['tm'],r['src'],r['prim'],r['err'],r['nval'],r['maxel'],r['maxsep'],r['bs']))
# is there a TM-free tell for genuine-low vs suppressed-normal? both have low maxel
print('\n=== TM-free separability check: genuine-low vs suppressed (both low maxel) ===')
gl=groups.get('genuine-low (TM<14, AMBIGUOUS)',[])
sup=groups.get('single-frame collapse (suppressed)',[])+groups.get('2-frame suppressed (reads low)',[])
for lab,rows in [('genuine-low',gl),('suppressed-normal',sup)]:
    if rows:
        print('  %-18s maxel %.1f+-%.1f  maxsep %.1f+-%.1f  nval %.1f  ball_speed %.0f+-%.0f'%(lab,
            np.mean([r['maxel'] for r in rows]),np.std([r['maxel'] for r in rows]),
            np.mean([r['maxsep'] for r in rows]),np.std([r['maxsep'] for r in rows]),
            np.mean([r['nval'] for r in rows]),np.mean([r['bs'] for r in rows]),np.std([r['bs'] for r in rows])))
