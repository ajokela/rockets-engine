import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.google.inject.Module;
import com.google.inject.util.Modules;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.l10n.DebugTranslator;
import info.openrocket.core.l10n.Translator;
import info.openrocket.core.models.wind.PinkNoiseWindModel;
import info.openrocket.core.motor.Manufacturer;
import info.openrocket.core.motor.Motor;
import info.openrocket.core.motor.MotorConfiguration;
import info.openrocket.core.motor.ThrustCurveMotor;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.rocketcomponent.FlightConfiguration;
import info.openrocket.core.rocketcomponent.InnerTube;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.simulation.FlightData;
import info.openrocket.core.simulation.SimulationOptions;
import info.openrocket.core.startup.Application;
import info.openrocket.core.startup.CoreModule;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.TestRockets;

/**
 * Flies OpenRocket's own Estes Alpha III fixture on its A8 and D21 test
 * motors with zero wind and prints both apogees plus the component masses,
 * so the Rust flight-parity tests can pin them. See README.md in this
 * directory.
 */
public class Apogee {

    private static Motor motor(String designation, double thrustPeak, double m0, double m1, double m2) {
        return new ThrustCurveMotor.Builder()
                .setManufacturer(Manufacturer.getManufacturer("Test"))
                .setDesignation(designation)
                .setDescription("").setCaseInfo("SU 18.0x70.0")
                .setMotorType(Motor.Type.SINGLE)
                .setStandardDelays(new double[] {})
                .setDiameter(0.018).setLength(0.070)
                .setTimePoints(new double[] { 0, 1, 2 })
                .setThrustPoints(new double[] { 0, thrustPeak, 0 })
                .setCGPoints(new Coordinate[] {
                        new Coordinate(0.035, 0, 0, m0),
                        new Coordinate(0.035, 0, 0, m1),
                        new Coordinate(0.035, 0, 0, m2) })
                .setDigest("test")
                .build();
    }

    private static Rocket preparedRocket() {
        Rocket rocket = TestRockets.makeEstesAlphaIII();
        java.util.List<RocketComponent> toRemove = new java.util.ArrayList<>();
        for (RocketComponent c : rocket) {
            if (c instanceof info.openrocket.core.rocketcomponent.TrapezoidFinSet) {
                // Our Barrowman modeler assumes streamlined fins; OpenRocket's
                // default SQUARE section adds ~(t/MAC)·A_fins/A_ref ≈ 1.0 of
                // pressure drag, which would compare nothing like-for-like.
                ((info.openrocket.core.rocketcomponent.TrapezoidFinSet) c)
                        .setCrossSection(info.openrocket.core.rocketcomponent.FinSet.CrossSection.ROUNDED);
            }
            if (c instanceof info.openrocket.core.rocketcomponent.ExternalComponent) {
                // SMOOTH = ~20 µm paint in OpenRocket's finish table,
                // matching our modeler's default roughness.
                ((info.openrocket.core.rocketcomponent.ExternalComponent) c)
                        .setFinish(info.openrocket.core.rocketcomponent.ExternalComponent.Finish.SMOOTH);
            }
            if (c instanceof info.openrocket.core.rocketcomponent.LaunchLug) {
                // Our geometry modeler has no launch lug; remove it from the
                // fixture so both sides model the same airframe.
                toRemove.add(c);
            }
        }
        for (RocketComponent c : toRemove) {
            c.getParent().removeChild(c);
        }
        return rocket;
    }

    private static double fly(Rocket rocket, Motor motor) throws Exception {
        InnerTube inner = null;
        for (RocketComponent c : rocket) {
            if (c instanceof InnerTube) {
                inner = (InnerTube) c;
            }
        }
        MotorConfiguration mc = new MotorConfiguration(inner, TestRockets.TEST_FCID_0);
        mc.setMotor(motor);
        mc.setEjectionDelay(1000.0); // chute never fires: apogee-only comparison
        inner.setMotorConfig(mc, TestRockets.TEST_FCID_0);

        Simulation sim = new Simulation(rocket);
        sim.setFlightConfigurationId(TestRockets.TEST_FCID_0);
        SimulationOptions opts = sim.getOptions();
        opts.setISAAtmosphere(true);
        opts.setTimeStep(0.05);
        PinkNoiseWindModel wind = opts.getAverageWindModel();
        wind.setAverage(0.0);
        wind.setStandardDeviation(0.0);
        sim.simulate();
        FlightData data = sim.getSimulatedData();
        System.out.printf("DETAIL maxV=%.2f maxMach=%.3f%n",
                data.getMaxVelocity(), data.getMaxMachNumber());
        info.openrocket.core.simulation.FlightDataBranch b = data.getBranch(0);
        java.util.List<Double> machs = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_MACH_NUMBER);
        java.util.List<Double> cds = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_DRAG_COEFF);
        java.util.List<Double> fric = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_FRICTION_DRAG_COEFF);
        java.util.List<Double> pres = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_PRESSURE_DRAG_COEFF);
        java.util.List<Double> base = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_BASE_DRAG_COEFF);
        java.util.List<Double> ts = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_TIME);
        double lastM = -1;
        for (int i = 0; i < machs.size(); i++) {
            double m = machs.get(i);
            if (m > 0.6 && (m - lastM > 0.15 || i == machs.size() - 1)) {
                lastM = m;
                System.out.printf("ROW M=%.2f cd=%.4f fric=%.4f pres=%.4f base=%.4f t=%.2f%n",
                        m, cds.get(i), fric.get(i), pres.get(i), base.get(i), ts.get(i));
                if (m > 1.8) break;
            }
        }
        return data.getMaxAltitude();
    }

    public static void main(String[] args) throws Exception {
        // Bootstrap the same way the unit tests do, minus JUnit.
        Module debugTranslator = new AbstractModule() {
            @Override
            protected void configure() {
                bind(Translator.class).toInstance(new DebugTranslator(null));
            }
        };
        Injector injector = Guice.createInjector(
                Modules.override(new CoreModule()).with(debugTranslator), new PluginModule());
        Application.setInjector(injector);

        Rocket rocket = preparedRocket();
        for (RocketComponent c : rocket) {
            double m = c.getComponentMass();
            if (m > 0.0) {
                System.out.printf("MASS %-24s %8.5f cg=%.4f%n",
                        c.getComponentName(), m, c.getComponentCG().x);
            }
        }

        // A8: 9 N-s triangle, 3.3 g propellant, 13.1 g casing. Subsonic
        // (Mach < 0.45) — pins the base drag model.
        double a8 = fly(preparedRocket(), motor("A8", 9.0, 0.0164, 0.0145, 0.0131));
        System.out.printf("APOGEE_A8 %.4f%n", a8);

        // D21: 32 N-s triangle, 9.6 g propellant, 15.4 g casing. Reaches
        // Mach 0.85 — pins the transonic pressure rise.
        double d21 = fly(preparedRocket(), motor("D21", 32.0, 0.025, 0.020, 0.0154));
        System.out.printf("APOGEE_D21 %.4f%n", d21);

        // G64: synthetic 128 N-s triangle (20 g propellant, 20 g casing) on
        // the same airframe. Reaches Mach ~1.7 — pins the supersonic
        // branch (K1 slope, nose wave drag, fin CP shift). Not a real
        // motor; parity needs identical inputs, not provenance.
        double g64 = fly(preparedRocket(), motor("G64", 128.0, 0.040, 0.030, 0.020));
        System.out.printf("APOGEE_G64 %.4f%n", g64);

        // Beta: the fixture's two-stage kit-bash (B4 sustainer over a D21
        // booster), pinning staging: separation and upper ignition times
        // and altitudes plus the stack's drag curve.
        Rocket beta = TestRockets.makeBeta();
        java.util.List<RocketComponent> betaRemove = new java.util.ArrayList<>();
        for (RocketComponent c : beta) {
            if (c instanceof info.openrocket.core.rocketcomponent.TrapezoidFinSet) {
                ((info.openrocket.core.rocketcomponent.TrapezoidFinSet) c)
                        .setCrossSection(info.openrocket.core.rocketcomponent.FinSet.CrossSection.ROUNDED);
            }
            if (c instanceof info.openrocket.core.rocketcomponent.ExternalComponent) {
                ((info.openrocket.core.rocketcomponent.ExternalComponent) c)
                        .setFinish(info.openrocket.core.rocketcomponent.ExternalComponent.Finish.SMOOTH);
            }
            if (c instanceof info.openrocket.core.rocketcomponent.LaunchLug) {
                betaRemove.add(c);
            }
        }
        for (RocketComponent c : betaRemove) {
            c.getParent().removeChild(c);
        }
        for (RocketComponent c : beta) {
            double m = c.getComponentMass();
            if (m > 0.0) {
                System.out.printf("BMASS %-28s %8.5f cg=%.4f%n",
                        c.getComponentName(), m, c.getComponentCG().x);
            }
        }
        Simulation bsim = new Simulation(beta);
        SimulationOptions bopts = bsim.getOptions();
        bopts.setISAAtmosphere(true);
        bopts.setTimeStep(0.05);
        PinkNoiseWindModel bwind = bopts.getAverageWindModel();
        bwind.setAverage(0.0);
        bwind.setStandardDeviation(0.0);
        bsim.simulate();
        FlightData bdata = bsim.getSimulatedData();
        System.out.printf("APOGEE_BETA %.4f%n", bdata.getMaxAltitude());
        info.openrocket.core.simulation.FlightDataBranch bb = bdata.getBranch(0);
        java.util.List<Double> bts = bb.get(info.openrocket.core.simulation.FlightDataType.TYPE_TIME);
        java.util.List<Double> balts = bb.get(info.openrocket.core.simulation.FlightDataType.TYPE_ALTITUDE);
        java.util.List<Double> bmachs = bb.get(info.openrocket.core.simulation.FlightDataType.TYPE_MACH_NUMBER);
        java.util.List<Double> bcds = bb.get(info.openrocket.core.simulation.FlightDataType.TYPE_DRAG_COEFF);
        for (info.openrocket.core.simulation.FlightEvent event : bb.getEvents()) {
            if (event.getType() == info.openrocket.core.simulation.FlightEvent.Type.IGNITION
                    || event.getType() == info.openrocket.core.simulation.FlightEvent.Type.BURNOUT
                    || event.getType() == info.openrocket.core.simulation.FlightEvent.Type.STAGE_SEPARATION) {
                double t = event.getTime();
                double alt = 0.0;
                for (int i = 1; i < bts.size(); i++) {
                    if (bts.get(i) >= t) {
                        alt = balts.get(i - 1);
                        break;
                    }
                }
                System.out.printf("EVENT %-18s t=%.3f alt=%.2f src=%s%n",
                        event.getType(), t, alt,
                        event.getSource() == null ? "" : event.getSource().getName());
            }
        }
        double lastM = -1;
        for (int i = 0; i < bmachs.size(); i++) {
            if (bmachs.get(i) - lastM > 0.1) {
                lastM = bmachs.get(i);
                System.out.printf("BCD M=%.2f cd=%.4f t=%.2f%n", lastM, bcds.get(i), bts.get(i));
            }
        }
    }
}
