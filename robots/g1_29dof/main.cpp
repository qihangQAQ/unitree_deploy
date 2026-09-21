#include "FSM/CtrlFSM.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FixStand.h"
#include "FSM/State_RLBase.h"
#include "State_Mimic.h"
#include "State_RLDepth.h"

#include <csignal>
#include <stdexcept>

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = std::make_shared<Keyboard>();

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void handle_signal(int)
{
    stop_requested = 1;
}

void lock_lowcmd()
{
    while (!FSMState::lowcmd->trylock()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void publish_initial_damping()
{
    const auto modes = param::config["FSM"]["Passive"]["mode"].as<std::vector<int>>();
    const auto damping = param::config["FSM"]["Passive"]["kd"].as<std::vector<float>>();
    if (modes.size() != damping.size() ||
        modes.size() > FSMState::lowcmd->msg_.motor_cmd().size()) {
        throw std::runtime_error("Passive mode/kd configuration has an invalid length");
    }

    lock_lowcmd();
    FSMState::lowcmd->msg_.mode_machine() = 5;
    {
        std::lock_guard<std::mutex> lock(FSMState::lowstate->mutex_);
        for (std::size_t i = 0; i < FSMState::lowcmd->msg_.motor_cmd().size(); ++i) {
            auto& motor = FSMState::lowcmd->msg_.motor_cmd()[i];
            motor.q() = FSMState::lowstate->msg_.motor_state()[i].q();
            motor.kp() = 0.0f;
            motor.dq() = 0.0f;
            motor.tau() = 0.0f;
            if (i < damping.size()) {
                motor.mode() = modes[i];
                motor.kd() = damping[i];
            }
        }
    }
    FSMState::lowcmd->unlockAndPublish();
}
}

void init_fsm_state()
{
    auto lowcmd_sub = std::make_shared<unitree::robot::g1::subscription::LowCmd>();
    usleep(0.2 * 1e6);
    if(!lowcmd_sub->isTimeout())
    {
        throw std::runtime_error(
            "Another process is publishing rt/lowcmd; refusing to start");
    }
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    spdlog::info("Waiting for connection to robot...");
    while (FSMState::lowstate->isTimeout() && !stop_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (stop_requested) {
        throw std::runtime_error("Startup interrupted while waiting for LowState");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    bool dds_initialized = false;
    try {
        // Load parameters
        auto vm = param::helper(argc, argv);

        std::cout << " --- Unitree Robotics --- \n";
        std::cout << "     G1-29dof Controller \n";

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        // Unitree DDS Config
        unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());
        dds_initialized = true;

        init_fsm_state();

        publish_initial_damping();
        if(!FSMState::lowcmd->check_mode_machine(FSMState::lowstate)) {
            throw std::runtime_error("Robot mode_machine does not match G1 29-DoF mode 5");
        }

        // Initialize FSM
        auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
        fsm->start();

        std::cout << "Press [L2 + Up] to enter FixStand mode.\n";
        std::cout << "Then press [R1 + X] for BlindWalk or [R1 + Y] for DepthWalk.\n";

        while (!stop_requested) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        spdlog::warn("Shutdown requested; switching to Passive damping mode");
        fsm->stop(std::chrono::milliseconds(500));
        fsm.reset();
        FSMState::lowcmd->stop();
        FSMState::lowcmd.reset();
        FSMState::lowstate.reset();
        unitree::robot::ChannelFactory::Instance()->Release();
        dds_initialized = false;
        return 0;
    } catch (const std::exception& error) {
        spdlog::critical("Controller startup/runtime failure: {}", error.what());
        if (FSMState::lowcmd) {
            FSMState::lowcmd->stop();
            FSMState::lowcmd.reset();
        }
        FSMState::lowstate.reset();
        if (dds_initialized) {
            unitree::robot::ChannelFactory::Instance()->Release();
        }
        return 1;
    }
}
