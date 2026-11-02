package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Field constants for the 2026 FRC game: REBUILT™ presented by Haas.
 *
 * Sources:
 *   - 2026 Game Manual (Section 5 ARENA)
 *   - 2026 Official Field Dimension Drawings, FE-2026 Rev A — Welded Perimeter
 *     (Sheets 3, 4, 7, 8, 11)
 *
 * Perimeter type: WELDED
 *
 * │  COORDINATE SYSTEM (Welded Perimeter — matches field drawing origin) │
 * │                                                                      │
 * │  Origin: corner where Blue Alliance Wall meets Scoring Table rail    │
 * │  +X  →  toward Red Alliance Station (field length axis)             │
 * │  +Y  →  away from Scoring Table     (field width axis)              │
 * │  +Z  →  up from carpet                                              │
 * │                                                                      │
 * │  X = 0       Blue Alliance Wall inside face                         │
 * │  X = 16.541  Red Alliance Wall inside face                          │
 * │  Y = 0       Scoring Table guardrail inside face                    │
 * │  Y = 8.071   Opposite (audience) guardrail inside face              │
 * └─────────────────────────────────────────────────────────────────────┘
 *
 * All linear dimensions are in METERS.
 * Conversion: 1 inch = 0.0254 m (exact).
 * Original inch values are shown in Javadoc comments for traceability.
 *
 * Position naming convention used throughout:
 *   X_CENTER  — centre of element along field-length axis
 *   X_FRONT   — element face CLOSER to field centre (away from its own alliance wall)
 *   X_BACK    — element face CLOSER to its own alliance wall (or touching guardrail for Trenches)
 *   Y_CENTER  — centre of element along field-width axis
 *   Y_FRONT   — element face CLOSER to Scoring Table (lower Y)
 *   Y_BACK    — element face CLOSER to audience guardrail (higher Y)
 *
 * NOTE: for Trenches the terms SCORING_SIDE / AUDIENCE_SIDE denote the two
 * separate trench structures (one per guardrail) within each alliance.
 */
public final class Field {

    // =========================================================================
    // FIELD OVERALL  (Welded Perimeter)
    // =========================================================================

    /** Overall field boundary dimensions (Welded Perimeter). */
    public static final class FieldDimensions {

        /** Interior length along X-axis (Blue wall → Red wall) — 651.22 in. */
        public static final double LENGTH = 16.541;

        /** Interior width along Y-axis (Scoring Table rail → Audience rail) — 317.69 in. */
        public static final double WIDTH = 8.071;

        /** Field centre X — 325.61 in. */
        public static final double X_CENTER = 8.271;

        /** Field centre Y — 158.84 in. */
        public static final double Y_CENTER = 4.035;

        /** Guardrail height (polycarbonate + extrusion system) — 20.0 in. */
        public static final double GUARDRAIL_HEIGHT = 0.508;

        /** Gate passthrough width when open — 38.0 in. */
        public static final double GATE_WIDTH = 0.965;
    }

    // =========================================================================
    // ALLIANCE WALL / DRIVER STATION  (Welded)
    // =========================================================================

    /** Alliance Wall and Driver Station dimensions (Welded Perimeter). */
    public static final class AllianceWall {

        /** Total wall height to top of uprights — 90.95 in. */
        public static final double HEIGHT = 2.310;

        /** Diamond plate top surface height from carpet — 35.95 in. */
        public static final double DIAMOND_PLATE_HEIGHT = 0.913;

        /** Polycarbonate panel height above diamond plate — 49.84 in. */
        public static final double POLY_HEIGHT = 1.266;

        /** Width of each driver station bay — 82.84 in. */
        public static final double DRIVER_STATION_WIDTH = 2.104;

        /** Driver station shelf width — 75.93 in. */
        public static final double DRIVER_STATION_SHELF_WIDTH = 1.929;

        /** Driver station shelf depth — 15.50 in. */
        public static final double DRIVER_STATION_SHELF_DEPTH = 0.394;

        /** Driver station shelf height from carpet — 42.00 in. */
        public static final double DRIVER_STATION_SHELF_HEIGHT = 1.067;
    }

    // =========================================================================
    // HUB  (GE-26300 — 1 per Alliance, centred on field width, 47 × 47 in footprint)
    //
    // Positions derived from AprilTag face coordinates on Sheet 11 (FE-2026 Rev A):
    //   Red  AT faces at X = 445.35 in (Blue-facing) and X = 492.88 in (Red-facing)
    //   Blue AT faces at X = 158.34 in (Blue-facing) and X = 205.87 in (Red-facing)
    //   Both alliances: Y faces at Y = 135.09 in (Scoring) and Y = 182.60 in (Audience)
    // =========================================================================

    /** RED Alliance Hub positional and size constants. */
    public static final class HubRed {

        // ── Size ──────────────────────────────────────────────────────────────
        /** Footprint width along X — 47.0 in. */
        public static final double WIDTH = 1.194;

        /** Footprint depth along Y — 47.0 in. */
        public static final double DEPTH = 1.194;

        /** Height of front edge of top hexagonal opening from carpet — 72.0 in. */
        public static final double OPENING_FRONT_HEIGHT = 1.829;

        /** Diameter of hexagonal opening — 41.7 in. */
        public static final double OPENING_DIAMETER = 1.059;

        // ── Positions ─────────────────────────────────────────────────────────
        /** Centre X — 469.12 in. */
        public static final double X_CENTER = 11.916;

        /**
         * X of the face nearest the Blue Alliance Wall (field-interior / "front" face).
         * Robots approach from lower X. — 445.35 in.
         */
        public static final double X_FRONT = 11.312;
        
        /**
         * X of the face nearest the Red Alliance Wall ("back" face). — 492.88 in.
         */
        public static final double X_BACK = 12.519;

        /** Centre Y — 158.84 in. */
        public static final double Y_CENTER = 4.035;

        /** Y of the face nearest the Scoring Table guardrail — 135.09 in. */
        public static final double Y_FRONT = 3.431;

        /** Y of the face nearest the Audience guardrail — 182.60 in. */
        public static final double Y_BACK = 4.638;

        /** AprilTag centre height on all four faces — 44.25 in. */
        public static final double APRILTAG_HEIGHT = 1.124;

        /** Distance from Red Alliance Wall to this Hub's X_BACK face — 158.34 in. */
        public static final double DISTANCE_FROM_WALL = 4.028;

        public static final Translation2d CENTER = new Translation2d(X_CENTER, Y_CENTER);
    }

    /** BLUE Alliance Hub positional and size constants. */
    public static final class HubBlue {

        // ── Size (identical to Red) ───────────────────────────────────────────
        /** Footprint width along X — 47.0 in. */
        public static final double WIDTH = 1.194;

        /** Footprint depth along Y — 47.0 in. */
        public static final double DEPTH = 1.194;

        /** Height of front edge of top hexagonal opening from carpet — 72.0 in. */
        public static final double OPENING_FRONT_HEIGHT = 1.829;

        /** Diameter of hexagonal opening — 41.7 in. */
        public static final double OPENING_DIAMETER = 1.059;

        // ── Positions ─────────────────────────────────────────────────────────
        /** Centre X — 182.11 in. */
        public static final double X_CENTER = 4.626;

        /** Centre Y — 158.84 in. */
        public static final double Y_CENTER = 4.035;

        public static final Translation2d CENTER = new Translation2d(X_CENTER, Y_CENTER);

        /**
         * X of the face nearest the Blue Alliance Wall ("back" face). — 158.34 in.
         */
        public static final double X_BACK = 4.022;

        /**
         * X of the face nearest the Red Alliance Wall (field-interior / "front" face).
         * Robots approach from higher X. — 205.87 in.
         */
        public static final double X_FRONT = 5.229;

        /** Y of the face nearest the Scoring Table guardrail — 135.09 in. */
        public static final double Y_FRONT = 3.431;

        /** Y of the face nearest the Audience guardrail — 182.60 in. */
        public static final double Y_BACK = 4.638;

        /** AprilTag centre height on all four faces — 44.25 in. */
        public static final double APRILTAG_HEIGHT = 1.124;

        /** Distance from Blue Alliance Wall to this Hub's X_BACK face — 158.34 in. */
        public static final double DISTANCE_FROM_WALL = 4.022;
    }

    // =========================================================================
    // BUMP  (GE-26100 — 4 total, 2 per Alliance, flanking each Hub in Y)
    //
    // Each Bump is 73 in wide (X) × 44.4 in deep (Y), centred on its Hub's X_CENTER.
    // Scoring-side Bump: Y_BACK = Hub Y_FRONT (135.09 in)
    // Audience-side Bump: Y_FRONT = Hub Y_BACK (182.60 in)
    // =========================================================================

    /** RED Alliance, Scoring-Table-side Bump. */
    public static final class BumpRedScoring {
        /** Width along X — 73.0 in. */
        public static final double WIDTH = 1.854;
        /** Depth along Y — 44.4 in. */
        public static final double DEPTH = 1.128;
        /** Peak height — 6.513 in. */
        public static final double HEIGHT = 0.165;
        /** Ramp angle (degrees). */
        public static final double RAMP_ANGLE_DEGREES = 15.0;
        /** HDPE ramp thickness — 0.5 in. */
        public static final double RAMP_THICKNESS = 0.013;

        /** Centre X — 469.12 in. */
        public static final double X_CENTER = 11.916;
        /** Field-facing (Blue-side) edge X — 432.62 in. */
        public static final double X_FRONT = 10.988;
        /** Alliance-wall-side (Red-side) edge X — 505.62 in. */
        public static final double X_BACK = 12.843;

        /** Centre Y — 112.89 in. */
        public static final double Y_CENTER = 2.867;
        /** Scoring Table guardrail side edge Y — 90.69 in. */
        public static final double Y_FRONT = 2.303;
        /** Hub-adjacent edge Y (= Hub Y_FRONT) — 135.09 in. */
        public static final double Y_BACK = 3.431;
    }

    /** RED Alliance, Audience-side Bump. */
    public static final class BumpRedAudience {
        /** Width along X — 73.0 in. */
        public static final double WIDTH = 1.854;
        /** Depth along Y — 44.4 in. */
        public static final double DEPTH = 1.128;
        /** Peak height — 6.513 in. */
        public static final double HEIGHT = 0.165;
        /** Ramp angle (degrees). */
        public static final double RAMP_ANGLE_DEGREES = 15.0;
        /** HDPE ramp thickness — 0.5 in. */
        public static final double RAMP_THICKNESS = 0.013;

        /** Centre X — 469.12 in. */
        public static final double X_CENTER = 11.916;
        /** Field-facing (Blue-side) edge X — 432.62 in. */
        public static final double X_FRONT = 10.988;
        /** Alliance-wall-side (Red-side) edge X — 505.62 in. */
        public static final double X_BACK = 12.843;

        /** Centre Y — 204.80 in. */
        public static final double Y_CENTER = 5.202;
        /** Hub-adjacent edge Y (= Hub Y_BACK) — 182.60 in. */
        public static final double Y_FRONT = 4.638;
        /** Audience guardrail side edge Y — 227.00 in. */
        public static final double Y_BACK = 5.766;
    }

    /** BLUE Alliance, Scoring-Table-side Bump. */
    public static final class BumpBlueScoring {
        /** Width along X — 73.0 in. */
        public static final double WIDTH = 1.854;
        /** Depth along Y — 44.4 in. */
        public static final double DEPTH = 1.128;
        /** Peak height — 6.513 in. */
        public static final double HEIGHT = 0.165;
        /** Ramp angle (degrees). */
        public static final double RAMP_ANGLE_DEGREES = 15.0;
        /** HDPE ramp thickness — 0.5 in. */
        public static final double RAMP_THICKNESS = 0.013;

        /** Centre X — 182.11 in. */
        public static final double X_CENTER = 4.626;
        /** Alliance-wall-side (Blue-side) edge X — 145.61 in. */
        public static final double X_BACK = 3.698;
        /** Field-facing (Red-side) edge X — 218.61 in. */
        public static final double X_FRONT = 5.553;

        /** Centre Y — 112.89 in. */
        public static final double Y_CENTER = 2.867;
        /** Scoring Table guardrail side edge Y — 90.69 in. */
        public static final double Y_FRONT = 2.303;
        /** Hub-adjacent edge Y (= Hub Y_FRONT) — 135.09 in. */
        public static final double Y_BACK = 3.431;
    }

    /** BLUE Alliance, Audience-side Bump. */
    public static final class BumpBlueAudience {
        /** Width along X — 73.0 in. */
        public static final double WIDTH = 1.854;
        /** Depth along Y — 44.4 in. */
        public static final double DEPTH = 1.128;
        /** Peak height — 6.513 in. */
        public static final double HEIGHT = 0.165;
        /** Ramp angle (degrees). */
        public static final double RAMP_ANGLE_DEGREES = 15.0;
        /** HDPE ramp thickness — 0.5 in. */
        public static final double RAMP_THICKNESS = 0.013;

        /** Centre X — 182.11 in. */
        public static final double X_CENTER = 4.626;
        /** Alliance-wall-side (Blue-side) edge X — 145.61 in. */
        public static final double X_BACK = 3.698;
        /** Field-facing (Red-side) edge X — 218.61 in. */
        public static final double X_FRONT = 5.553;

        /** Centre Y — 204.80 in. */
        public static final double Y_CENTER = 5.202;
        /** Hub-adjacent edge Y (= Hub Y_BACK) — 182.60 in. */
        public static final double Y_FRONT = 4.638;
        /** Audience guardrail side edge Y — 227.00 in. */
        public static final double Y_BACK = 5.766;
    }

    // =========================================================================
    // TRENCH  (GE-26200 — 4 total: 2 per Alliance × Scoring side + Audience side)
    //
    // Each Trench: 65.65 in wide (X) × 47.0 in deep (Y) × 40.25 in tall.
    // Extends from the guardrail (Y=0 or Y=317.69) inward 47 in.
    // X span centred on the corresponding Hub's X_CENTER.
    // Clearance opening underneath: 50.34 in wide × 22.25 in tall.
    // =========================================================================

    /** RED Alliance, Scoring-Table-side Trench (against Y = 0 guardrail). */
    public static final class TrenchRedScoring {
        /** Overall width along X — 65.65 in. */
        public static final double WIDTH = 1.668;
        /** Overall depth into field along Y — 47.0 in. */
        public static final double DEPTH = 1.194;
        /** Overall height — 40.25 in. */
        public static final double HEIGHT = 1.022;
        /** Under-arm clearance width — 50.34 in. */
        public static final double CLEARANCE_WIDTH = 1.279;
        /** Under-arm clearance height — 22.25 in. */
        public static final double CLEARANCE_HEIGHT = 0.565;

        /** Centre X — 469.12 in. */
        public static final double X_CENTER = 11.916;
        /** Blue-side edge X — 436.29 in. */
        public static final double X_FRONT = 11.082;
        /** Red-side edge X — 501.94 in. */
        public static final double X_BACK = 12.749;

        /** Centre Y — 23.50 in. */
        public static final double Y_CENTER = 0.597;
        /** Scoring Table guardrail face (at guardrail) — 0.00 in. */
        public static final double Y_FRONT = 0.000;
        /** Field-facing edge Y — 47.00 in. */
        public static final double Y_BACK = 1.194;

        /** AprilTag centre height — 35.0 in. */
        public static final double APRILTAG_HEIGHT = 0.889;
    }

    /** RED Alliance, Audience-side Trench (against Y = 317.69 in guardrail). */
    public static final class TrenchRedAudience {
        /** Overall width along X — 65.65 in. */
        public static final double WIDTH = 1.668;
        /** Overall depth into field along Y — 47.0 in. */
        public static final double DEPTH = 1.194;
        /** Overall height — 40.25 in. */
        public static final double HEIGHT = 1.022;
        /** Under-arm clearance width — 50.34 in. */
        public static final double CLEARANCE_WIDTH = 1.279;
        /** Under-arm clearance height — 22.25 in. */
        public static final double CLEARANCE_HEIGHT = 0.565;

        /** Centre X — 469.12 in. */
        public static final double X_CENTER = 11.916;
        /** Blue-side edge X — 436.29 in. */
        public static final double X_FRONT = 11.082;
        /** Red-side edge X — 501.94 in. */
        public static final double X_BACK = 12.749;

        /** Centre Y — 294.19 in. */
        public static final double Y_CENTER = 7.472;
        /** Field-facing edge Y — 270.69 in. */
        public static final double Y_FRONT = 6.876;
        /** Audience guardrail face — 317.69 in. */
        public static final double Y_BACK = 8.069;

        /** AprilTag centre height — 35.0 in. */
        public static final double APRILTAG_HEIGHT = 0.889;
    }

    /** BLUE Alliance, Scoring-Table-side Trench (against Y = 0 guardrail). */
    public static final class TrenchBlueScoring {
        /** Overall width along X — 65.65 in. */
        public static final double WIDTH = 1.668;
        /** Overall depth into field along Y — 47.0 in. */
        public static final double DEPTH = 1.194;
        /** Overall height — 40.25 in. */
        public static final double HEIGHT = 1.022;
        /** Under-arm clearance width — 50.34 in. */
        public static final double CLEARANCE_WIDTH = 1.279;
        /** Under-arm clearance height — 22.25 in. */
        public static final double CLEARANCE_HEIGHT = 0.565;

        /** Centre X — 182.11 in. */
        public static final double X_CENTER = 4.626;
        /** Blue-side edge X — 149.28 in. */
        public static final double X_BACK = 3.792;
        /** Red-side edge X — 214.93 in. */
        public static final double X_FRONT = 5.459;

        /** Centre Y — 23.50 in. */
        public static final double Y_CENTER = 0.597;
        /** Scoring Table guardrail face — 0.00 in. */
        public static final double Y_FRONT = 0.000;
        /** Field-facing edge Y — 47.00 in. */
        public static final double Y_BACK = 1.194;

        /** AprilTag centre height — 35.0 in. */
        public static final double APRILTAG_HEIGHT = 0.889;
    }

    /** BLUE Alliance, Audience-side Trench (against Y = 317.69 in guardrail). */
    public static final class TrenchBlueAudience {
        /** Overall width along X — 65.65 in. */
        public static final double WIDTH = 1.668;
        /** Overall depth into field along Y — 47.0 in. */
        public static final double DEPTH = 1.194;
        /** Overall height — 40.25 in. */
        public static final double HEIGHT = 1.022;
        /** Under-arm clearance width — 50.34 in. */
        public static final double CLEARANCE_WIDTH = 1.279;
        /** Under-arm clearance height — 22.25 in. */
        public static final double CLEARANCE_HEIGHT = 0.565;

        /** Centre X — 182.11 in. */
        public static final double X_CENTER = 4.626;
        /** Blue-side edge X — 149.28 in. */
        public static final double X_BACK = 3.792;
        /** Red-side edge X — 214.93 in. */
        public static final double X_FRONT = 5.459;

        /** Centre Y — 294.19 in. */
        public static final double Y_CENTER = 7.472;
        /** Field-facing edge Y — 270.69 in. */
        public static final double Y_FRONT = 6.876;
        /** Audience guardrail face — 317.69 in. */
        public static final double Y_BACK = 8.069;

        /** AprilTag centre height — 35.0 in. */
        public static final double APRILTAG_HEIGHT = 0.889;
    }

    // =========================================================================
    // DEPOT  (GE-26600 — 1 per Alliance, along Alliance Wall, 42 × 27 in)
    //
    // Located between the Tower and the field interior along the Alliance Wall.
    // X_BACK touches the Alliance Wall. Y position from Sheet 3 reference dims.
    // =========================================================================

    /** RED Alliance Depot. */
    public static final class DepotRed {
        /** Width along Y — 42.0 in. */
        public static final double WIDTH = 1.067;
        /** Depth along X (into field) — 27.0 in. */
        public static final double DEPTH = 0.686;
        /** Steel barrier height (un-secured) — 1.0 in. */
        public static final double BARRIER_HEIGHT = 0.025;
        /** Steel barrier height with hook fastener — 1.125 in. */
        public static final double BARRIER_HEIGHT_WITH_FASTENER = 0.029;
        /** Max FUEL capacity. */
        public static final int FUEL_CAPACITY = 24;

        /** Centre X — 637.72 in. */
        public static final double X_CENTER = 16.198;
        /** Field-facing edge X (front face) — 624.22 in. */
        public static final double X_FRONT = 15.855;
        /** Alliance Wall (back face) X — 651.22 in. */
        public static final double X_BACK = 16.541;

        /** Centre Y — 153.63 in. */
        public static final double Y_CENTER = 3.902;
        /** Scoring-Table-side edge Y — 132.63 in. */
        public static final double Y_FRONT = 3.369;
        /** Audience-side edge Y — 174.63 in. */
        public static final double Y_BACK = 4.436;
    }

    /** BLUE Alliance Depot. */
    public static final class DepotBlue {
        /** Width along Y — 42.0 in. */
        public static final double WIDTH = 1.067;
        /** Depth along X (into field) — 27.0 in. */
        public static final double DEPTH = 0.686;
        /** Steel barrier height (un-secured) — 1.0 in. */
        public static final double BARRIER_HEIGHT = 0.025;
        /** Steel barrier height with hook fastener — 1.125 in. */
        public static final double BARRIER_HEIGHT_WITH_FASTENER = 0.029;
        /** Max FUEL capacity. */
        public static final int FUEL_CAPACITY = 24;

        /** Centre X — 13.50 in. */
        public static final double X_CENTER = 0.343;
        /** Alliance Wall (back face) X — 0.00 in. */
        public static final double X_BACK = 0.000;
        /** Field-facing edge X (front face) — 27.00 in. */
        public static final double X_FRONT = 0.686;

        /** Centre Y — 164.06 in. */
        public static final double Y_CENTER = 4.167;
        /** Scoring-Table-side edge Y — 143.06 in. */
        public static final double Y_FRONT = 3.634;
        /** Audience-side edge Y — 185.06 in. */
        public static final double Y_BACK = 4.701;
    }

    // =========================================================================
    // TOWER  (GE-26500 — 1 per Alliance, integrated into Alliance Wall)
    //
    // Overall: 49.25 in wide (Y) × 45.0 in deep (X) × 78.25 in tall.
    // Tower Base: 39.0 in (Y) × 45.18 in (X) plate extending from Alliance Wall.
    // Uprights: two posts, 72.1 in tall, 32.25 in centre-to-centre span (Y).
    // Three RUNGS at 27, 45, and 63 in from carpet.
    // Positions from AprilTag face coords (Sheet 11, FE-2026 Rev A):
    //   Red  AT15/16: X = 650.90 in, Y = 170.22 / 153.22 in → centre Y = 161.72 in
    //   Blue AT31/32: X = 0.32 in,  Y = 147.47 / 164.47 in → centre Y = 155.97 in
    // =========================================================================

    /** RED Alliance Tower. */
    public static final class TowerRed {
        /** Overall width along Y — 49.25 in. */
        public static final double WIDTH = 1.251;
        /** Overall depth along X — 45.0 in. */
        public static final double DEPTH = 1.143;
        /** Overall height — 78.25 in. */
        public static final double HEIGHT = 1.988;

        /** Tower Base width along Y — 39.0 in. */
        public static final double BASE_WIDTH = 0.991;
        /** Tower Base depth along X — 45.18 in. */
        public static final double BASE_DEPTH = 1.148;
        /** Tower Base edge height — 0.25 in nominal. */
        public static final double BASE_EDGE_HEIGHT = 0.006;

        /** Each Upright height — 72.1 in. */
        public static final double UPRIGHT_HEIGHT = 1.831;
        /** Upright wall thickness — 1.5 in. */
        public static final double UPRIGHT_THICKNESS = 0.038;
        /** Upright depth front-to-back — 3.5 in. */
        public static final double UPRIGHT_DEPTH = 0.089;
        /** Centre-to-centre Upright span along Y — 32.25 in. */
        public static final double UPRIGHT_SPAN = 0.819;

        /** Rung pipe OD (1-1/4 in Sch 40) — 1.66 in. */
        public static final double RUNG_PIPE_OD = 0.042;
        /** Spacing between each Rung — 18.0 in. */
        public static final double RUNG_SPACING = 0.457;
        /** LOW Rung height from carpet — 27.0 in. */
        public static final double RUNG_LOW_HEIGHT = 0.686;
        /** MID Rung height from carpet — 45.0 in. */
        public static final double RUNG_MID_HEIGHT = 1.143;
        /** HIGH Rung height from carpet — 63.0 in. */
        public static final double RUNG_HIGH_HEIGHT = 1.600;

        /** AprilTag centre height on Tower Wall — 21.75 in. */
        public static final double APRILTAG_HEIGHT = 0.553;

        /** Centre X — 628.72 in. */
        public static final double X_CENTER = 15.969;
        /** Field-facing (front) face X — 606.22 in. */
        public static final double X_FRONT = 15.398;
        /** Alliance Wall (back) face X — 651.22 in. */
        public static final double X_BACK = 16.541;

        /** Centre Y (from AprilTag data) — 161.72 in. */
        public static final double Y_CENTER = 4.108;
        /** Scoring-Table-side edge Y — 137.09 in. */
        public static final double Y_FRONT = 3.482;
        /** Audience-side edge Y — 186.34 in. */
        public static final double Y_BACK = 4.733;
    }

    /** BLUE Alliance Tower. */
    public static final class TowerBlue {
        /** Overall width along Y — 49.25 in. */
        public static final double WIDTH = 1.251;
        /** Overall depth along X — 45.0 in. */
        public static final double DEPTH = 1.143;
        /** Overall height — 78.25 in. */
        public static final double HEIGHT = 1.988;

        /** Tower Base width along Y — 39.0 in. */
        public static final double BASE_WIDTH = 0.991;
        /** Tower Base depth along X — 45.18 in. */
        public static final double BASE_DEPTH = 1.148;
        /** Tower Base edge height — 0.25 in nominal. */
        public static final double BASE_EDGE_HEIGHT = 0.006;

        /** Each Upright height — 72.1 in. */
        public static final double UPRIGHT_HEIGHT = 1.831;
        /** Upright wall thickness — 1.5 in. */
        public static final double UPRIGHT_THICKNESS = 0.038;
        /** Upright depth front-to-back — 3.5 in. */
        public static final double UPRIGHT_DEPTH = 0.089;
        /** Centre-to-centre Upright span along Y — 32.25 in. */
        public static final double UPRIGHT_SPAN = 0.819;

        /** Rung pipe OD (1-1/4 in Sch 40) — 1.66 in. */
        public static final double RUNG_PIPE_OD = 0.042;
        /** Spacing between each Rung — 18.0 in. */
        public static final double RUNG_SPACING = 0.457;
        /** LOW Rung height from carpet — 27.0 in. */
        public static final double RUNG_LOW_HEIGHT = 0.686;
        /** MID Rung height from carpet — 45.0 in. */
        public static final double RUNG_MID_HEIGHT = 1.143;
        /** HIGH Rung height from carpet — 63.0 in. */
        public static final double RUNG_HIGH_HEIGHT = 1.600;

        /** AprilTag centre height on Tower Wall — 21.75 in. */
        public static final double APRILTAG_HEIGHT = 0.553;

        /** Centre X — 22.50 in. */
        public static final double X_CENTER = 0.572;
        /** Alliance Wall (back) face X — 0.00 in. */
        public static final double X_BACK = 0.000;
        /** Field-facing (front) face X — 45.00 in. */
        public static final double X_FRONT = 1.143;

        /** Centre Y (from AprilTag data) — 155.97 in. */
        public static final double Y_CENTER = 3.962;
        /** Scoring-Table-side edge Y — 131.34 in. */
        public static final double Y_FRONT = 3.336;
        /** Audience-side edge Y — 180.59 in. */
        public static final double Y_BACK = 4.587;
    }

    // =========================================================================
    // OUTPOST  (GE-26000 — 1 per Alliance, along Alliance Wall / Human Player zone)
    //
    // Outpost Area: 71 in wide (Y) × 134 in deep (X).
    // Positions from AprilTag face coords (Sheet 11, FE-2026 Rev A):
    //   Red  AT13/14: X = 650.92 in, Y = 291.47 / 274.47 in → centre Y = 282.97 in
    //   Blue AT29/30: X = 0.30 in,  Y = 26.22  / 43.22  in → centre Y = 34.72  in
    // =========================================================================

    /** RED Alliance Outpost. */
    public static final class OutpostRed {
        /** Outpost Area width along Y — 71.0 in. */
        public static final double AREA_WIDTH = 1.803;
        /** Outpost Area depth along X — 134.0 in. */
        public static final double AREA_DEPTH = 3.404;

        /** CHUTE opening width — 31.8 in. */
        public static final double CHUTE_OPENING_WIDTH = 0.808;
        /** CHUTE opening height — 7.0 in. */
        public static final double CHUTE_OPENING_HEIGHT = 0.178;
        /** CHUTE opening bottom from floor — 28.1 in. */
        public static final double CHUTE_OPENING_BOTTOM = 0.714;

        /** CORRAL opening bottom from floor — 1.88 in. */
        public static final double CORRAL_OPENING_BOTTOM = 0.048;
        /** CORRAL width — 35.8 in. */
        public static final double CORRAL_WIDTH = 0.909;
        /** CORRAL depth — 37.6 in. */
        public static final double CORRAL_DEPTH = 0.955;
        /** CORRAL polycarbonate wall height — 8.13 in. */
        public static final double CORRAL_WALL_HEIGHT = 0.207;
        /** Divider pipe OD (1-1/4 in Sch 40) — 1.66 in. */
        public static final double CORRAL_DIVIDER_PIPE_OD = 0.042;

        /** AprilTag centre height on Outpost wall — 21.75 in. */
        public static final double APRILTAG_HEIGHT = 0.553;
        /** CHUTE max FUEL capacity. */
        public static final int CHUTE_FUEL_CAPACITY = 24;

        /** Centre X — 633.72 in (AT face at 650.92 in, structure ~35 in deep). */
        public static final double X_CENTER = 16.096;
        /** Field-facing (front) face X — 616.22 in. */
        public static final double X_FRONT = 15.652;
        /** Alliance Wall (back) face X — 651.22 in. */
        public static final double X_BACK = 16.541;

        /** Centre Y (from AprilTag data) — 282.97 in. */
        public static final double Y_CENTER = 7.187;
        /** Scoring-Table-side edge Y — 256.97 in. */
        public static final double Y_FRONT = 6.527;
        /** Audience-side edge Y — 308.97 in. */
        public static final double Y_BACK = 7.848;
    }

    /** BLUE Alliance Outpost. */
    public static final class OutpostBlue {
        /** Outpost Area width along Y — 71.0 in. */
        public static final double AREA_WIDTH = 1.803;
        /** Outpost Area depth along X — 134.0 in. */
        public static final double AREA_DEPTH = 3.404;

        /** CHUTE opening width — 31.8 in. */
        public static final double CHUTE_OPENING_WIDTH = 0.808;
        /** CHUTE opening height — 7.0 in. */
        public static final double CHUTE_OPENING_HEIGHT = 0.178;
        /** CHUTE opening bottom from floor — 28.1 in. */
        public static final double CHUTE_OPENING_BOTTOM = 0.714;

        /** CORRAL opening bottom from floor — 1.88 in. */
        public static final double CORRAL_OPENING_BOTTOM = 0.048;
        /** CORRAL width — 35.8 in. */
        public static final double CORRAL_WIDTH = 0.909;
        /** CORRAL depth — 37.6 in. */
        public static final double CORRAL_DEPTH = 0.955;
        /** CORRAL polycarbonate wall height — 8.13 in. */
        public static final double CORRAL_WALL_HEIGHT = 0.207;
        /** Divider pipe OD (1-1/4 in Sch 40) — 1.66 in. */
        public static final double CORRAL_DIVIDER_PIPE_OD = 0.042;

        /** AprilTag centre height on Outpost wall — 21.75 in. */
        public static final double APRILTAG_HEIGHT = 0.553;
        /** CHUTE max FUEL capacity. */
        public static final int CHUTE_FUEL_CAPACITY = 24;

        /** Centre X — 17.50 in. */
        public static final double X_CENTER = 0.445;
        /** Alliance Wall (back) face X — 0.00 in. */
        public static final double X_BACK = 0.000;
        /** Field-facing (front) face X — 35.00 in. */
        public static final double X_FRONT = 0.889;

        /** Centre Y (from AprilTag data) — 34.72 in. */
        public static final double Y_CENTER = 0.882;
        /** Scoring-Table-side edge Y — 8.72 in. */
        public static final double Y_FRONT = 0.221;
        /** Audience-side edge Y — 60.72 in. */
        public static final double Y_BACK = 1.542;
    }

    public static final class DELIVERY{
        public static final Translation2d DELIVERY_POINT1 = new Translation2d(12.6619, 4.023 - 2.5);
        public static final Translation2d DELIVERY_POINT2 = new Translation2d(12.6619, 4.023 + 2.5);    
    }

    // =========================================================================
    // GAME PIECE — FUEL
    // =========================================================================

    /** Fuel game piece counts. */
    public static final class Fuel {
        /** Total FUEL placed on field at match start (standard configuration). */
        public static final int STANDARD_COUNT = 456;
        /** Additional FUEL (pre-load / reserve). */
        public static final int EXTRA_COUNT = 48;
    }

    // =========================================================================
    // ZONE / AREA DIMENSIONS  (Welded Perimeter, Section 5.3 of Game Manual)
    // =========================================================================

    /** Named field zones and their boundary dimensions. */
    public static final class Zones {
        /** Alliance Zone depth (Alliance Wall → Robot Starting Line) — 158.6 in. */
        public static final double ALLIANCE_ZONE_DEPTH = 4.028;
        /** Alliance Zone width (full field width) — 317.7 in. */
        public static final double ALLIANCE_ZONE_WIDTH = 8.070;

        /** Alliance Area depth — 134.0 in. */
        public static final double ALLIANCE_AREA_DEPTH = 3.404;
        /** Alliance Area width (approx.) — 360.0 in. */
        public static final double ALLIANCE_AREA_WIDTH = 9.144;

        /** Neutral Zone depth — 283.0 in. */
        public static final double NEUTRAL_ZONE_DEPTH = 7.188;
        /** Neutral Zone width — 317.7 in. */
        public static final double NEUTRAL_ZONE_WIDTH = 8.070;

        /** Outpost Area width — 71.0 in. */
        public static final double OUTPOST_AREA_WIDTH = 1.803;
        /** Outpost Area depth — 134.0 in. */
        public static final double OUTPOST_AREA_DEPTH = 3.404;

        /** Human Starting Line offset from Alliance Wall bottom tube, near edge — 24.0 in. */
        public static final double HUMAN_STARTING_LINE_OFFSET = 0.610;

        /**
         * Robot Starting Line X offset from Red Alliance Wall face.
         * = Alliance Zone depth — 158.84 in from Red wall.
         */
        public static final double ROBOT_STARTING_LINE_RED_X = FieldDimensions.LENGTH - 4.028; // 12.513 m
        /**
         * Robot Starting Line X offset from Blue Alliance Wall face.
         * = Alliance Zone depth — 158.84 in from Blue wall.
         */
        public static final double ROBOT_STARTING_LINE_BLUE_X = 4.028;

        /** Field centre line X (midpoint of field length) — 325.61 in. */
        public static final double CENTER_LINE_X = 8.271;
    }

    // =========================================================================
    // WELDED PERIMETER — KEY CONTROLLED DIMENSIONS  (Sheets 7 & 8, FE-2026 Rev A)
    // =========================================================================

    /** Controlled (toleranced) dimensions from the official Welded Perimeter drawings. */
    public static final class WeldedPerimeter {
        /** Inside field length (Blue Wall → Red Wall) — 651.22 in. */
        public static final double INSIDE_LENGTH = 16.541;
        /** Inside field width (Scoring Table → Audience) — 317.69 in. */
        public static final double INSIDE_WIDTH = 8.071;

        /** Half-field width (from each guardrail to centre) — 158.84 in. */
        public static final double HALF_WIDTH = 4.035;

        /** Distance from Alliance Wall face to Robot Starting Line tape, near edge — 46.88 in. */
        public static final double TAPE_LINE_OFFSET = 1.191;
        /** Distance from tape line to Outpost plastic face — 5.13 in. */
        public static final double TAPE_TO_OUTPOST = 0.130;
        /** Distance from tape line to rear of driver station frame — 27.00 in. */
        public static final double TAPE_TO_DS_FRAME = 0.686;
        /** Distance from tape line to edge of diamond plate — 15.50 in. */
        public static final double TAPE_TO_DIAMOND_PLATE = 0.394;
        /** Distance from base plate to edge of diamond plate — 15.50 in. */
        public static final double BASE_TO_DIAMOND_PLATE = 0.394;
        /** Alliance tape line to nearest Trench/Hub face — 24.00 in. */
        public static final double TAPE_TO_TRENCH_HUB = 0.610;

        /** Controlled Trench under-arm clearance width — 50.34 in. */
        public static final double TRENCH_CLEARANCE_WIDTH = 1.279;
        /** Controlled Trench/Bump span width along guardrail axis — 73.00 in. */
        public static final double TRENCH_BUMP_WIDTH = 1.854;
        /** Alliance station outer span (from field edge to DS outer face) — 22.25 in. */
        public static final double DS_OUTER_SPAN = 0.565;
    }
}