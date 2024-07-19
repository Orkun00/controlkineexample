#include <signal.h>
#include <ncurses.h>
#include <map>
#include <string>
#include <vector>
#include <sstream>
#include <memory>
#include <thread>
#include <chrono>
#include "../include/Component.hpp"
#include "../include/Dashboard.hpp"
#include "../include/Button.hpp"
#include "../include/Input.hpp"
#include "../include/Menu.hpp"
#include "../include/tmotor.hpp"

int main(int argc, char **argv) {
    float gear_ratio;
    std::string can_interface;
    try {
        gear_ratio = std::stof(argv[1]);
        can_interface = argv[2];
        if (can_interface != "vcan0" && can_interface != "can0") throw std::runtime_error("Invalid CAN interface");
    } catch(const std::exception& e) {
        gear_ratio = 1.0f;
        can_interface = "vcan0";
    }

    initscr();
    atexit((void (*)()) endwin);

    keypad(stdscr, TRUE);
    raw();
    curs_set(0);
    noecho();
    refresh();

    init_colors();
    paint_scr();

    int motor_id = 0;
    { // Input motor ID
        InputBufferHex input("Motor ID", (COLS-30)/2, LINES/2, 30, 5);
        attron(COLOR_PAIR(2));
        mvprintw(LINES/2+6, (COLS-54)/2, "Please enter the motor ID using hexadecimal notation.");
        attroff(COLOR_PAIR(2));
        refresh();
        input.mount();
        InputUpdate packet('\n');
        input.update(&packet);
        motor_id = input.m_value;
        if (char c = getch() == 'q') return 0;
        input.unmount();
        clear();
        refresh();
    }

    std::shared_ptr<TMotor::AKManager> manager12 = std::make_shared<TMotor::AKManager>(12);
    std::shared_ptr<TMotor::AKManager> manager13 = std::make_shared<TMotor::AKManager>(13);
    std::shared_ptr<TMotor::AKManager> manager14 = std::make_shared<TMotor::AKManager>(14);

    manager12->connect(can_interface.c_str());
    manager13->connect(can_interface.c_str());
    manager14->connect(can_interface.c_str());

    bool shutdown = false;
    Menu menu(1+COLS/5, 0, 4*COLS/5, (COLS)/(5*2), &shutdown, manager12, gear_ratio);
    Dashboard dashboard(0, 0, COLS/5, (COLS)/(5*2), motor_id);
    
    menu.mount();
    dashboard.mount();
    menu.focus();
    dashboard.focus();

    std::thread dashboard_updater([&shutdown, &dashboard, &manager12, &manager13, &manager14, gear_ratio] {
        while (!shutdown) {
            AKPacket motor_packet12(
                manager12->getCurrent(),
                manager12->getPosition(),
                manager12->getVelocity(),
                gear_ratio,
                manager12->getTemperature(),
                manager12->getFault()
            );
            AKPacket motor_packet13(
                manager13->getCurrent(),
                manager13->getPosition(),
                manager13->getVelocity(),
                gear_ratio,
                manager13->getTemperature(),
                manager13->getFault()
            );
            AKPacket motor_packet14(
                manager14->getCurrent(),
                manager14->getPosition(),
                manager14->getVelocity(),
                gear_ratio,
                manager14->getTemperature(),
                manager14->getFault()
            );

            dashboard.update(&motor_packet12);
            dashboard.update(&motor_packet13);
            dashboard.update(&motor_packet14);

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    InputUpdate packet('0');
    while (((packet.key_in = getch()) != 'q') && (!shutdown)) {
        menu.update(&packet);
    }
    shutdown = true;
    dashboard.unmount();
    menu.unmount();
    dashboard_updater.join();

    return 0;
}
