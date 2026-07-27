import { renderToString } from 'react-dom/server';
import { describe, expect, it } from 'vitest';
import type { Shot } from '../types/shot';
import { ShotDisplay } from './ShotDisplay';

const experimentalShot: Shot = {
  ball_speed_mph: 105.2,
  club_speed_mph: 79.4,
  smash_factor: 1.32,
  estimated_carry_yards: 132,
  carry_range: [125, 139],
  club: 'pitching_wedge',
  timestamp: '2026-07-17T12:00:00Z',
  peak_magnitude: 42,
  launch_angle_vertical: null,
  launch_angle_horizontal: null,
  launch_angle_confidence: null,
  angle_source: null,
  club_angle_deg: -3.2,
  club_path_deg: 0.8,
  spin_axis_deg: -1.1,
  spin_rpm: 8750,
  spin_confidence: 0.36,
  spin_quality: 'experimental',
  spin_source: 'measured',
  spin_method: 'multitaper_ungated',
  spin_multipath_fade_hz: 48.2,
  carry_spin_adjusted: null,
};

describe('ShotDisplay', () => {
  it('labels ungated spin as experimental without confidence dots', () => {
    const html = renderToString(<ShotDisplay shot={experimentalShot} />);

    expect(html).toContain('8,750');
    expect(html).toContain('metric-card__confidence--experimental');
    expect(html).toContain('experimental');
    expect(html).not.toContain('metric-card__confidence-dots');
  });

  // The IWR6843 supplies these three: club path and horizontal launch from the
  // TX2 baseline, attack angle from the vertical array. Each card is
  // conditional on its value being non-null, because the radar rejects a
  // measurement far more often than it produces one -- so both branches need
  // pinning, or a card can silently stop rendering.
  describe('radar angle cards', () => {
    const withAngles: Shot = {
      ...experimentalShot,
      club_angle_deg: -4.3,
      club_path_deg: 2.6,
      launch_angle_horizontal: -1.8,
      launch_angle_confidence: 0.83,
      angle_source: 'radar',
    };

    it('renders club path with an explicit sign', () => {
      const html = renderToString(<ShotDisplay shot={withAngles} />);

      expect(html).toContain('Club Path');
      expect(html).toContain('+2.6');
    });

    it('renders a negative club path without a spurious plus', () => {
      const html = renderToString(
        <ShotDisplay shot={{ ...withAngles, club_path_deg: -2.6 }} />,
      );

      expect(html).toContain('Club Path');
      expect(html).toContain('-2.6');
      expect(html).not.toContain('+-2.6');
    });

    it('renders horizontal launch and attack angle', () => {
      const html = renderToString(<ShotDisplay shot={withAngles} />);

      expect(html).toContain('H. Launch');
      expect(html).toContain('-1.8');
      expect(html).toContain('Club AoA');
      expect(html).toContain('-4.3');
    });

    it('omits each card when the radar produced no measurement', () => {
      const html = renderToString(
        <ShotDisplay
          shot={{
            ...withAngles,
            club_path_deg: null,
            launch_angle_horizontal: null,
            club_angle_deg: null,
          }}
        />,
      );

      expect(html).not.toContain('Club Path');
      expect(html).not.toContain('H. Launch');
      expect(html).not.toContain('Club AoA');
    });
  });
});
