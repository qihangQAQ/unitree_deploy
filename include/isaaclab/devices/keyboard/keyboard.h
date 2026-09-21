#pragma once

#include <atomic>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

/** Maintain an optional, thread-safe keyboard input stream for local testing. */
class Keyboard
{
public:
    Keyboard()
    {
        terminal_available_ = ::isatty(fileno(stdin)) &&
                              tcgetattr(fileno(stdin), &old_settings_) == 0;
        if (!terminal_available_) {
            return;
        }

        new_settings_ = old_settings_;
        new_settings_.c_lflag &= (~ICANON & ~ECHO);
        start_key();
        thread_running_.store(true);
        read_thread_ = std::thread([this] {
            while (thread_running_.load()) {
                if (reading_enabled_.load()) {
                    read_once();
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
        });
    }

    ~Keyboard()
    {
        thread_running_.store(false);
        reading_enabled_.store(false);
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        pause_key();
    }

    void update()
    {
        std::lock_guard<std::mutex> lock(key_mutex_);
        if (key_ != last_key_) {
            on_pressed = !key_.empty();
            on_released = key_.empty();
        } else {
            on_pressed = false;
            on_released = false;
        }
        last_key_ = key_;
    }

    std::string key() const
    {
        std::lock_guard<std::mutex> lock(key_mutex_);
        return key_;
    }

    std::string getString(const std::string& slogan)
    {
        reading_enabled_.store(false);
        pause_key();
        std::cout << slogan << std::endl;
        std::string value;
        std::getline(std::cin, value);
        start_key();
        return value;
    }

    bool on_pressed = false;
    bool on_released = false;

private:
    void set_key(std::string value)
    {
        std::lock_guard<std::mutex> lock(key_mutex_);
        key_ = std::move(value);
    }

    void read_once()
    {
        char input = '\0';
        if (!read_char(input)) {
            set_key("");
            return;
        }
        if (input != '\033') {
            set_key(std::string(1, input));
            return;
        }

        char bracket = '\0';
        char direction = '\0';
        if (!read_char(bracket) || bracket != '[' || !read_char(direction)) {
            set_key("");
            return;
        }
        switch (direction) {
        case 'A': set_key("up"); break;
        case 'B': set_key("down"); break;
        case 'C': set_key("right"); break;
        case 'D': set_key("left"); break;
        default: set_key(""); break;
        }
    }

    bool read_char(char& value)
    {
        fd_set descriptors;
        FD_ZERO(&descriptors);
        FD_SET(fileno(stdin), &descriptors);
        timeval timeout{0, 80000};

        const int selected = select(fileno(stdin) + 1, &descriptors, nullptr, nullptr, &timeout);
        if (selected <= 0) {
            return false;
        }
        return ::read(fileno(stdin), &value, 1) == 1;
    }

    void pause_key()
    {
        reading_enabled_.store(false);
        if (terminal_available_) {
            tcsetattr(fileno(stdin), TCSANOW, &old_settings_);
        }
    }

    void start_key()
    {
        if (terminal_available_) {
            tcsetattr(fileno(stdin), TCSANOW, &new_settings_);
            reading_enabled_.store(true);
        }
    }

    bool terminal_available_ = false;
    std::atomic_bool thread_running_{false};
    std::atomic_bool reading_enabled_{false};
    std::thread read_thread_;
    mutable std::mutex key_mutex_;
    std::string key_;
    std::string last_key_;
    termios old_settings_{};
    termios new_settings_{};
};
