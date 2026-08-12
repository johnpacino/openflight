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

    it('labels rejected club candidates as experimental with their reason', () => {
      const html = renderToString(
        <ShotDisplay
          shot={{
            ...withAngles,
            club_angle_deg: null,
            club_path_deg: null,
            experimental_attack_angle_deg: -4.9,
            experimental_attack_angle_status: 'rejected_azimuth_fit',
            experimental_club_path_deg: 5.8,
            experimental_club_path_status: 'rejected_phase_span',
          }}
        />,
      );

      expect(html).toContain('Club AoA');
      expect(html).toContain('-4.9');
      expect(html).toContain('Club Path');
      expect(html).toContain('+5.8');
      expect(html).toContain('rejected: azimuth fit');
      expect(html).toContain('rejected: phase span');
      expect(html.match(/metric-card__confidence--experimental/g)).toHaveLength(3);
    });

    it('keeps status-only experimental club metrics visible', () => {
      const html = renderToString(
        <ShotDisplay
          shot={{
            ...withAngles,
            club_angle_deg: null,
            club_path_deg: null,
            experimental_attack_angle_status: 'rejected_no_club_track',
            experimental_club_path_status: 'rejected_no_club_track',
          }}
        />,
      );

      expect(html).toContain('Club AoA');
      expect(html).toContain('Club Path');
      expect(html.match(/rejected: no club track/g)).toHaveLength(2);
      expect(html.match(/metric-card__confidence--experimental/g)).toHaveLength(3);
    });

    it('shows independent confidence dots for camera-fused experimental delivery', () => {
      const html = renderToString(
        <ShotDisplay
          shot={{
            ...withAngles,
            club_angle_deg: null,
            club_path_deg: null,
            experimental_fused_attack_angle_deg: -4.2,
            experimental_fused_attack_angle_confidence: 'medium',
            experimental_fused_club_path_deg: 3.1,
            experimental_fused_club_path_confidence: 'high',
            experimental_fused_status: 'approach_mixed',
          }}
        />,
      );

      expect(html).toContain('camera fused (experimental)');
      expect(html).toContain('metric-card__confidence--medium');
      expect(html).toContain('metric-card__confidence--high');
      // Horizontal launch also carries dots in this fixture.
      expect(html.match(/metric-card__confidence-dots/g)).toHaveLength(3);
      // Experimental spin is the third confidence label in this fixture.
      expect(html.match(/metric-card__confidence-label">experimental/g)).toHaveLength(3);
    });

    it('does not expose rejected radar candidates after camera fusion was attempted', () => {
      const html = renderToString(
        <ShotDisplay
          shot={{
            ...withAngles,
            club_angle_deg: null,
            club_path_deg: null,
            experimental_attack_angle_deg: -32.2,
            experimental_attack_angle_status: 'candidate_out_of_bounds',
            experimental_club_path_deg: 130.9,
            experimental_club_path_status: 'candidate_out_of_bounds',
            experimental_fused_status: 'rejected_no_impact',
            experimental_fused_attack_angle_confidence: 'withheld',
            experimental_fused_club_path_confidence: 'withheld',
          }}
        />,
      );

      expect(html).not.toContain('-32.2');
      expect(html).not.toContain('+130.9');
      expect(html).toContain('rejected: no impact');
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
