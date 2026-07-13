import json, numpy as np
recs=json.load(open('/tmp/of-ttv/recs.json'))
def primary(r):
    if r['pos'] is not None: return r['pos']
    if r['sing'] is not None: return r['sing']
    return r['live']
for r in recs:
    p=primary(r); r['prim']=p; r['err']=(abs(p-r['tm']) if p is not None else None)
def is_t1(r,S=12,E=12): return r['nval']>=2 and r['maxsep']>=S and r['maxel']>=E and r['pos'] is not None

print('=== Q1: does Tier-1 only keep "ideal" (normal-launch) shots? ===')
t1=[r for r in recs if is_t1(r)]; rej=[r for r in recs if not is_t1(r) and r['err'] is not None]
print('  TIER-1   TM launch-angle range: %.1f - %.1f  (mean %.1f)'%(min(r['tm'] for r in t1),max(r['tm'] for r in t1),np.mean([r['tm'] for r in t1])))
print('  REJECT   TM launch-angle range: %.1f - %.1f  (mean %.1f)'%(min(r['tm'] for r in rej),max(r['tm'] for r in rej),np.mean([r['tm'] for r in rej])))
print('\n  All shots with TM launch < 14 deg (the punches/tops/fats/low):')
print('    club    ts        TM   prim   err   maxel maxsep  in Tier1?')
for r in sorted([x for x in recs if x['tm']<14 and x['err'] is not None],key=lambda r:r['tm']):
    print('    %-7s %s %5.1f %6.1f %+6.1f  %5.1f %5.1f    %s'%(r['club'][:7],r['ts'],r['tm'],r['prim'],r['prim']-r['tm'],r['maxel'],r['maxsep'],'YES' if is_t1(r) else 'no'))
# how many low shots were actually measured accurately but excluded?
lowacc=[r for r in recs if r['tm']<14 and r['err'] is not None and r['err']<=1.5 and not is_t1(r)]
print('\n  -> %d low-launch shots are measured within 1.5 deg but EXCLUDED from Tier-1'%len(lowacc))
print('     (selector cannot tell "genuinely low" from "suppressed low", so it defers both)')

print('\n=== Q2: does driver get ANY Tier-1 shots as we lower the gate? ===')
drv=[r for r in recs if r['club']=='driver']
for S,E in [(12,12),(12,9),(9,9),(9,6),(6,6)]:
    sel=[r for r in drv if is_t1(r,S,E)]
    print('  maxsep>=%2d & maxel>=%2d : driver Tier-1 n=%d %s'%(S,E,len(sel),
        ('  '+', '.join('%s(err%+.1f)'%(r['ts'],r['prim']-r['tm']) for r in sel) if sel else '')))
print('\n  driver shots that even HAVE a position fit (pos != None):')
for r in drv:
    if r['pos'] is not None:
        print('    %s  TM %.1f  pos %.1f (err %+.1f)  maxsep %.1f  maxel %.1f  nval %d'%(r['ts'],r['tm'],r['pos'],r['pos']-r['tm'],r['maxsep'],r['maxel'],r['nval']))
print('  (binding constraint for driver = maxsep & having a pos fit at all, NOT maxel)')
