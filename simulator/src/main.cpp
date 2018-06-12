#include <iostream>
#include <functional>
#include <vector>

//***************************

class Pose3D {
    // TODO
};
class Direction3D {
    // TODO
};
class Duration {
    // TODO
};
class TimePoint {
    // TODO
public:
    TimePoint operator+=(Duration const&)
    {
        // TODO
        return {};
    }

    friend bool operator == (TimePoint const&, TimePoint const&) {
        // TODO
        return true;
    }
};


class Transformation {
    // TODO
public:
    Pose3D operator()(Pose3D) {
        return {
            // TODO
        };
    }
};

class AngleVelocity {
    // TODO
};
class Acceleration {
    // TODO
};



//***************************
class KinematicState {
    // TODO
    TimePoint m_time;
public:
    KinematicState(TimePoint time)
        : m_time{std::move(time)}
    {}

    TimePoint getTime() const { return m_time; }

    KinematicState transformed(Transformation) const {
        // TODO
        return *this;
    }
};

class TrajectoryIterator;
class Trajectory;

class TrajectoryRange {
    friend class TrajectoryIterator;
    TimePoint m_startTime;
    TimePoint m_endTime;
    Duration m_step;
    Trajectory const& m_trajectory;

public:
    TrajectoryRange(TimePoint start, TimePoint stop, Duration step, Trajectory const& trajectory);

    TrajectoryIterator begin() const;

    TrajectoryIterator end() const;

    Duration getStep() const;
};

class Trajectory {
public:
    KinematicState evaluateAtTime(TimePoint) const;

    TrajectoryRange getRange(Duration) const;

    Trajectory transformed(Transformation const&) const;
};

class TrajectoryIterator : public std::iterator<
                        std::input_iterator_tag,       // iterator_category
                        KinematicState,                // value_type
                        Duration,                      // difference_type
                        KinematicState*,               // pointer
                        KinematicState&                // reference
                                      >
{
    TimePoint m_time;
    TrajectoryRange const& m_range;

public:
    TrajectoryIterator(TimePoint time, TrajectoryRange const& range)
        : m_time{std::move(time)}
        , m_range{range}
    {}

    TrajectoryIterator& operator++() {
        m_time += m_range.getStep();
        return *this;
    }

    TrajectoryIterator operator++(int) {
        auto copy(*this);
        ++(*this);
        return copy;
    }

    bool operator==(TrajectoryIterator const& other) const {
        return m_time == other.m_time;
    }

    bool operator!=(TrajectoryIterator const& other) const {
        return !(*this == other);
    }

    KinematicState operator*() const {
        return m_range.m_trajectory.evaluateAtTime(m_time);
    }
};

TrajectoryRange::TrajectoryRange(TimePoint start, TimePoint stop, Duration step, Trajectory const& trajectory)
    : m_startTime{std::move(start)}
    , m_endTime{std::move(stop)}
    , m_step{std::move(step)}
    , m_trajectory{trajectory}
{}

TrajectoryIterator TrajectoryRange::begin() const {
    return { m_startTime, *this };
}

TrajectoryIterator TrajectoryRange::end() const {
    return { m_endTime, *this };
}

Duration TrajectoryRange::getStep() const { return m_step; }

KinematicState Trajectory::evaluateAtTime(TimePoint time) const {
    return {time};
}

TrajectoryRange Trajectory::getRange(Duration step) const {
    return {TimePoint{}, TimePoint{}, step, *this};
}

Trajectory Trajectory::transformed(Transformation const&) const {
    return {};
}

class TrajectoryBuilder {
    // TODO
public:
    Trajectory build() const {
        // TODO
        return {};
    }
    TrajectoryBuilder& initialPose(Pose3D) { return *this; }
    TrajectoryBuilder& atTime(TimePoint) { return *this; }
    TrajectoryBuilder& accelerate(Duration, Acceleration) { return *this; }
    TrajectoryBuilder& rotateSystem(Direction3D, AngleVelocity) { return *this; }
};

//***************************

class SimulatorConfig {
    // TODO
    Duration m_timeStep;
public:
    SimulatorConfig(Duration timeStep)
        : m_timeStep{std::move(timeStep)}
    {}

    Duration getTimeStep() const { return m_timeStep; }
};

class SimulatorConfigBuilder {
    // TODO
    Duration m_timeStep;

public:
    SimulatorConfig build() const {
        return { m_timeStep };
    }

    SimulatorConfigBuilder& setTimeStep(Duration step) {
        m_timeStep = step;
        return *this;
    }

};


//***************************
class EnvironmentConfig {
    // TODO
};

class EnvironmentConfigBuilder {
    // TODO
public:
    EnvironmentConfig build() const {
        // TODO
        return {};
    }

    EnvironmentConfigBuilder& addSphere() {
        // TODO
        return *this;
    }

    EnvironmentConfigBuilder& addBox() {
        // TODO
        return *this;
    }
};
//***************************

class ImuSensorConfig {
    // TODO
public:
    ImuSensorConfig(std::string){}
};
class Laser2dSensorConfig {
    // TODO
public:
    Laser2dSensorConfig(std::string) {}
};

//***************************
class SystemConfig {
    // TODO
};

class SystemConfigBuilder {
    // TODO
public:
    SystemConfig build() const {
        // TODO
        return {};
    }
    SystemConfigBuilder& addImu(ImuSensorConfig) {
        // TODO
        return *this;
    }
    SystemConfigBuilder& add2dLaser(Laser2dSensorConfig) {
        // TODO
        return *this;
    }
};

//***************************
class Environment {
    // TODO
};

class EnvironmentFactory {
public:
    Environment create(EnvironmentConfig) const {
        return {};
    }
};

class RawSensorDataCollector {
    // TODO
public:
    void addImuSample(TimePoint) {
        // TODO
    }
    void add2dLaserSamples(TimePoint) {
        // TODO
    }
};


//***************************

class SimulationSetup {};
class Simulation {
    std::function<void(RawSensorDataCollector&)> m_behavior;

public:
    Simulation(std::function<void(RawSensorDataCollector&)> behavior)
        : m_behavior{ std::move(behavior) }
    {}

    void run(RawSensorDataCollector& dataCollector) {
        m_behavior(dataCollector);
    }
};

class SimulationBuilder {
    std::function<void(RawSensorDataCollector&)> m_behavior;

public:
    SimulationBuilder& setBehavior(std::function<void(RawSensorDataCollector&)> behavior) {
        m_behavior = std::move(behavior);
        return *this;
    }
    Simulation build() const {
        return { m_behavior };
    }
};


//***************************
class AbstractSensorImpl {
public:
    virtual Simulation
    sample(Environment& env, KinematicState const& state) const = 0;

    virtual Simulation
    sampleAt(Environment& env, Trajectory const& trajectory, Duration const& period) const {
        const auto behavior = [&env,&trajectory,&period, this](RawSensorDataCollector& collector) {
            for(auto const& state : trajectory.getRange(period)) {
                sample(env, state).run(collector);
            }
        };
        return SimulationBuilder()
            .setBehavior(behavior)
            .build();

    }
};

class AbstractSensor {
    std::unique_ptr<AbstractSensorImpl> m_impl;

public:
    AbstractSensor(std::unique_ptr<AbstractSensorImpl> impl)
        : m_impl{std::move(impl)}
    {}

    Simulation
    sample(Environment& env, KinematicState const& state) const {
        return m_impl->sample(env, state);
    };

    Simulation
    sampleAt(Environment& env, Trajectory const& trajectory, Duration const& period) const {
        return m_impl->sampleAt(env, trajectory, period);
    }
};


//***************************
class ImuSensorImpl : public AbstractSensorImpl {
public:
    virtual Simulation
    sample(Environment& env, KinematicState const& state) const {
        auto const behavior = [&state](RawSensorDataCollector& collector) {
            /* auto sample =
                { Acceleration{ state.getAcceleration() + gravity }
                , AngleVelocity{ state.getAngularVelocityFrom() }
                }; */
            collector.addImuSample(state.getTime()/*sample*/);
        };
        return SimulationBuilder()
            .setBehavior(behavior)
            .build();
    }

};


class SystemSensorImpl : public AbstractSensorImpl {
    struct SensorConfig {
        AbstractSensor sensor;
        Transformation trafo;
        Duration period;
    };
    std::vector<SensorConfig> m_sensors;

public:
    virtual Simulation
    sample(Environment& env, KinematicState const& state) const override {
        auto const behavior = [&state,&env,this](RawSensorDataCollector& collector) {
            for(auto const& sensorConfig : m_sensors) {
                auto simulation = sensorConfig.sensor.sample(
                    env,
                    state.transformed(sensorConfig.trafo));
                simulation.run(collector);
            }
        };
        return SimulationBuilder()
            .setBehavior(behavior)
            .build();
    }

    virtual Simulation
    sampleAt(Environment& env, Trajectory const& trajectory, Duration const& period) const override {
        auto const behavior = [&env,&trajectory,&period, this](RawSensorDataCollector& collector) {
            for(auto const& config : m_sensors) {
                auto simulation = config.sensor.sampleAt(
                    env,
                    trajectory.transformed(config.trafo),
                    config.period);
                simulation.run(collector);
            }
        };
        return SimulationBuilder()
            .setBehavior(behavior)
            .build();
    }
};

class SystemFactory {
public:
    AbstractSensor create(SystemConfig) {
        return AbstractSensor{std::make_unique<SystemSensorImpl>()};
    }
};


class Simulator {
    SimulatorConfig m_simConfig;
    Environment m_env;

public:
    Simulator(SimulatorConfig simConfig, Environment env)
        : m_simConfig{std::move(simConfig)}
        , m_env{std::move(env)}
    {}

    std::function<void(RawSensorDataCollector&)>
    simulate(AbstractSensor const& sensor, Trajectory const& trajectory) {
        return [&sensor,&trajectory,this](RawSensorDataCollector& collector) {
            for(auto const& state : trajectory.getRange(m_simConfig.getTimeStep())) {
                auto simulation = sensor.sample(m_env, state);
                simulation.run(collector);
            }
        };
    }
};
//***************************

class SimulatorFactory {
public:
    Simulator create(
        SimulatorConfig simConfig,
        Environment env)
    {
        return {simConfig, env};
    }
};


//***************************
int main()
{


    auto simulatorConfig =
        SimulatorConfigBuilder()
            /* ... */
            .build();

    auto environmentConfig =
        EnvironmentConfigBuilder()
            .addSphere(/*...*/)
            .addBox(/*...*/)
            .build();

    auto systemConfig =
        SystemConfigBuilder()
            .addImu(ImuSensorConfig{"imu1" /*...*/})
            .add2dLaser(Laser2dSensorConfig{"laser2d_1" /*...*/})
            .build();

    auto environment = EnvironmentFactory().create(environmentConfig);
    auto simulator = SimulatorFactory().create(
        simulatorConfig,
        environment);

    auto trajectory =
        TrajectoryBuilder()
            .initialPose(Pose3D{})
            .atTime(TimePoint{})
            .accelerate(Duration{}, Acceleration{})
            .atTime(TimePoint{})
            .rotateSystem(Direction3D{}, AngleVelocity{})
            .build();

    auto system = SystemFactory().create(systemConfig);
    auto simulation = simulator.simulate(system, trajectory);

    std::cout << "hello world from simulator" << std::endl;
}


/*
void test() {
    System system;
    Trajectory trajectory;
    Environment environment;

    ScanFileWriter scanFileWriter;
    SimulatorConfig simConfig;

    auto simulator = Simulator{simConfig,environment};
    simulator.simulationOf(system, trajectory).runWith(scanFileWriter);
}*/
