#include <iostream>
#include <functional>
#include <vector>

//***************************

class Pose3D {};
class Direction3D {};
class Duration {};
class TimePoint {
public:
    TimePoint operator+=(Duration const&)
    {
        return {};
    }

    friend bool operator == (TimePoint const&, TimePoint const&) {
        return true;
    }
};


class Transformation {
public:
    Pose3D operator()(Pose3D) {
        return {};
    }
};

class AngleVelocity {};
class Acceleration {};



//***************************
class KinematicState {
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

class TrajectoryBuilder {
public:
    Trajectory build() const {
        return {};
    }
    TrajectoryBuilder& initialPose(Pose3D) { return *this; }
    TrajectoryBuilder& atTime(TimePoint) { return *this; }
    TrajectoryBuilder& accelerate(Duration, Acceleration) { return *this; }
    TrajectoryBuilder& rotateSystem(Direction3D, AngleVelocity) { return *this; }
};

//***************************

class SimulatorConfig {
    Duration m_timeStep;
public:
    SimulatorConfig(Duration timeStep)
        : m_timeStep{std::move(timeStep)}
    {}

    Duration getTimeStep() const { return m_timeStep; }
};

class SimulatorConfigBuilder {
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
class EnvironmentConfig {};

class EnvironmentConfigBuilder {
public:
    EnvironmentConfig build() const {
        return {};
    }

    EnvironmentConfigBuilder& addSphere() {
        return *this;
    }

    EnvironmentConfigBuilder& addBox() {
        return *this;
    }
};
//***************************

class ImuSensorConfig { public: ImuSensorConfig(std::string){} };
class Laser2dSensorConfig { public: Laser2dSensorConfig(std::string) {} };

//***************************
class SystemConfig {};

class SystemConfigBuilder {
public:
    SystemConfig build() const {
        return {};
    }
    SystemConfigBuilder& addImu(ImuSensorConfig) {
        return *this;
    }
    SystemConfigBuilder& add2dLaser(Laser2dSensorConfig) {
        return *this;
    }
};

//***************************
class Environment {};

class EnvironmentFactory {
public:
    Environment create(EnvironmentConfig) const {
        return {};
    }
};

class RawSensorDataCollector {
public:
    void addImuSample(TimePoint) {}
    void add2dLaserSamples(TimePoint) {}
};



//***************************
class AbstractSensorImpl {
public:
    virtual std::function<void(RawSensorDataCollector&)>
    sample(
        Environment& env, KinematicState const& state) const = 0;
};

class AbstractSensor {
    std::unique_ptr<AbstractSensorImpl> m_impl;

public:
    AbstractSensor(std::unique_ptr<AbstractSensorImpl> impl)
        : m_impl{std::move(impl)}
    {}

    std::function<void(RawSensorDataCollector&)>
    sample(Environment& env, KinematicState const& state) const
    {
        return m_impl->sample(env, state);
    };
};

//***************************
class ImuSensorImpl : public AbstractSensorImpl {
public:
    virtual std::function<void(RawSensorDataCollector&)>
    sample(Environment& env, KinematicState const& state) const {
        return [&state](RawSensorDataCollector& collector) {
            /* auto sample =
                { Acceleration{ state.getAcceleration() + gravity }
                , AngleVelocity{ state.getAngularVelocityFrom() }
                }; */
            collector.addImuSample(state.getTime()/*, sample*/);
        };
    }
};


class SystemSensorImpl : public AbstractSensorImpl {
    struct SensorConfig {
        AbstractSensor sensor;
        Transformation trafo;
    };
    std::vector<SensorConfig> m_sensors;

public:
    virtual std::function<void(RawSensorDataCollector&)>
    sample(Environment& env, KinematicState const& state) const override {
        return [&state,&env,this](RawSensorDataCollector& collector) {
            for(auto const& sensorConfig : m_sensors) {
                sensorConfig.sensor.sample(env, state.transformed(sensorConfig.trafo))(collector);
            }
        };
    }
};

class SystemFactory {
public:
    AbstractSensor create(SystemConfig) {
        return AbstractSensor{std::make_unique<SystemSensorImpl>()};
    }
};

//***************************

class SimulationSetup {};
class Simulation {};

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
                sensor.sample(m_env, state)(collector);
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
