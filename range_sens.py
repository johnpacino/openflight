import math
# 12:17:28 single-frame solve, as measured
distance_ft = 5.0
above = -4.0/12.0          # ball_above_radar_ft = -0.333
el_meas = 5.14             # measured elevation, deg
rng_meas = 8.94            # measured F1B range, ft
t_ms = 30.1                # tau-corrected time after impact
bs_mph = 99.8              # raw OPS ball speed
TM = 25.5
MPH_TO_FTS = 1.4666667

def la_from(rng, el_deg):
    el = math.radians(el_deg)
    bx = rng*math.cos(el) - distance_ft
    by = rng*math.sin(el) - above
    if bx <= 0: return float('nan')
    return math.degrees(math.atan2(by, bx))

print("== current solve ==")
print("  range=%.2f el=%.2f  -> LA %.2f  (TM %.1f)"%(rng_meas, el_meas, la_from(rng_meas, el_meas), TM))

print("\n== Q: hold el=5.14, vary range ==")
for r in [6.0, 6.5, 7.0, 7.05, 7.5, 8.0, 8.94, 9.5, 10.0]:
    print("  range=%5.2f -> LA %6.2f"%(r, la_from(r, el_meas)))
# range that yields TM exactly (el fixed)
# solve (r*sin - above) = tan(TM)*(r*cos - distance)
tn = math.tan(math.radians(TM)); el=math.radians(el_meas)
r_star = (tn*distance_ft - above)/(tn*math.cos(el) - math.sin(el))
print("  -> range giving exactly %.1f deg (el fixed) = %.2f ft"%(TM, r_star))

print("\n== forward check: where SHOULD a %.1f-deg ball be at +%.1f ms? =="%(TM, t_ms))
v = bs_mph*MPH_TO_FTS                  # ft/s
t = t_ms/1000.0
g = 32.174
vH = v*math.cos(math.radians(TM)); vV = v*math.sin(math.radians(TM))
horiz = vH*t                            # along-ground from tee
height = vV*t - 0.5*g*t*t               # above tee
x = distance_ft + horiz                 # rel radar (horizontal)
y = above + height                      # rel radar (vertical)
rng_pred = math.hypot(x, y)
el_pred = math.degrees(math.atan2(y, x))
print("  predicted range=%.2f ft  el=%.2f deg"%(rng_pred, el_pred))
print("  MEASURED  range=%.2f ft  el=%.2f deg"%(rng_meas, el_meas))
print("  -> range error %+.2f ft | elevation error %+.2f deg"%(rng_meas-rng_pred, el_meas-el_pred))

print("\n== if we instead TRUST range=8.94 and fix only el ==")
# what el at the measured range gives TM?
# (r*sin - above) = tan(TM)*(r*cos - distance) -> solve el
# r*sin(el) - tan*r*cos(el) = tan*distance - above ... transcendental; just scan
best=None
for e in [x*0.01 for x in range(0,2000)]:
    if abs(la_from(rng_meas, e)-TM) < (abs(la_from(rng_meas, best)-TM) if best else 9e9):
        best=e
print("  el giving %.1f deg at range 8.94 = %.2f deg  (measured 5.14, so el is ~%.1f deg low)"%(TM, best, best-el_meas))
