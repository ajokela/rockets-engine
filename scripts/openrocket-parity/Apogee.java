import com.google.inject.AbstractModule;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.google.inject.Module;
import com.google.inject.util.Modules;
import info.openrocket.core.document.Simulation;
import info.openrocket.core.l10n.DebugTranslator;
import info.openrocket.core.l10n.Translator;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.startup.Application;
import info.openrocket.core.startup.CoreModule;
import info.openrocket.core.models.wind.PinkNoiseWindModel;
import info.openrocket.core.motor.Manufacturer;
import info.openrocket.core.motor.Motor;
import info.openrocket.core.motor.MotorConfiguration;
import info.openrocket.core.motor.ThrustCurveMotor;
import info.openrocket.core.rocketcomponent.FlightConfiguration;
import info.openrocket.core.rocketcomponent.InnerTube;
import info.openrocket.core.rocketcomponent.Rocket;
import info.openrocket.core.rocketcomponent.RocketComponent;
import info.openrocket.core.simulation.FlightData;
import info.openrocket.core.simulation.SimulationOptions;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.TestRockets;

/**
 * Flies OpenRocket's own Estes Alpha III fixture on its D21 test motor with
 * zero wind and prints the apogee plus the component masses, so the Rust
 * flight-parity test can pin both. See README.md in this directory.
 */
public class Apogee {
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

        Rocket rocket = TestRockets.makeEstesAlphaIII();
        FlightConfiguration config = rocket.getSelectedConfiguration();

        // The fixture's A8: 9 N-s triangle, 18 mm x 70 mm, 3.3 g propellant.
        // Kept deliberately slow (Mach < 0.3) so the comparison stays in the
        // subsonic regime our Barrowman modeler covers; the D21 reaches
        // Mach 0.8, where OpenRocket's pre-transonic rise (which we clamp)
        // dominates the difference.
        Motor a8 = new ThrustCurveMotor.Builder()
                .setManufacturer(Manufacturer.getManufacturer("Estes"))
                .setDesignation("A8")
                .setDescription("SU Black Powder").setCaseInfo("SU 18.0x70.0")
                .setMotorType(Motor.Type.SINGLE)
                .setStandardDelays(new double[] {})
                .setDiameter(0.018).setLength(0.070)
                .setTimePoints(new double[] { 0, 1, 2 })
                .setThrustPoints(new double[] { 0, 9, 0 })
                .setCGPoints(new Coordinate[] {
                        new Coordinate(0.035, 0, 0, 0.0164),
                        new Coordinate(0.035, 0, 0, 0.0145),
                        new Coordinate(0.035, 0, 0, 0.0131) })
                .setDigest("digest A8 test")
                .build();

        InnerTube inner = null;
        java.util.List<RocketComponent> toRemove = new java.util.ArrayList<>();
        for (RocketComponent c : rocket) {
            if (c instanceof InnerTube) {
                inner = (InnerTube) c;
            }
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
        MotorConfiguration mc = new MotorConfiguration(inner, TestRockets.TEST_FCID_0);
        mc.setMotor(a8);
        mc.setEjectionDelay(1000.0); // chute never fires: apogee-only comparison
        inner.setMotorConfig(mc, TestRockets.TEST_FCID_0);

        for (RocketComponent c : rocket) {
            double m = c.getComponentMass();
            if (m > 0.0) {
                System.out.printf("MASS %-24s %8.5f cg=%.4f%n",
                        c.getComponentName(), m, c.getComponentCG().x);
            }
        }

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
        System.out.printf("APOGEE %.4f%n", data.getMaxAltitude());
        System.out.printf("MAXV %.4f%n", data.getMaxVelocity());

        info.openrocket.core.simulation.FlightDataBranch b = data.getBranch(0);
        java.util.List<Double> ts = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_TIME);
        java.util.List<Double> alts = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_ALTITUDE);
        java.util.List<Double> vels = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_VELOCITY_TOTAL);
        java.util.List<Double> cds = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_DRAG_COEFF);
        java.util.List<Double> fric = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_FRICTION_DRAG_COEFF);
        java.util.List<Double> pres = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_PRESSURE_DRAG_COEFF);
        java.util.List<Double> base = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_BASE_DRAG_COEFF);
        java.util.List<Double> masses = b.get(info.openrocket.core.simulation.FlightDataType.TYPE_MASS);
        for (int i = 0; i < ts.size(); i++) {
            if (ts.get(i) <= 3.0 || i % 40 == 0) {
                System.out.printf("ROW t=%.2f alt=%.2f v=%.2f cd=%.4f fric=%.4f pres=%.4f base=%.4f m=%.5f%n",
                        ts.get(i), alts.get(i), vels.get(i),
                        cds.get(i), fric.get(i), pres.get(i), base.get(i), masses.get(i));
            }
        }
    }
}
