/**
 * @file terminal.hpp
 * @author Nilusink
 * @brief defines a few useful things for terminal output
 * @date 2024-12-12
 * 
 * @copyright Copyright Nilusink (c) 2024
 * 
 */
#pragma once
#include <cstdint>


// help: https://michurin.github.io/xterm256-color-picker/
#define FG_COLOR(n) "\u001b[38;5;"#n"m"


/**
 * @brief Console Colors - console modifiers (colors, style, ...)
 * 
 */
namespace cc
{
    /**
     * @brief Control - styling
     * 
     */
    namespace ctrl
    {
        const char ENDC[] = "\033[0m";
        const char BOLD[] = "\033[1m";
        const char FAINT[] = "\033[2m";
        const char ITALIC[] = "\033[3m";
        const char UNDERLINE[] = "\033[4m";
        const char SLOW_BLINK[] = "\033[5m";
        const char RAPID_BLINK[] = "\033[6m";
        const char INVERSE[] = "\033[7m";
        const char CONCEAL[] = "\033[8m";
        const char CROSSED[] = "\033[9m";
        const char FRAKTUR[] = "\033[20";
        const char NORMAL1[] = "\033[22";
        const char NORMAL2[] = "\033[23";
        const char UNDERLINE_OFF[] = "\033[24";
        const char BLINK_OFF[] = "\033[25";
        const char INVERSE_OFF[] = "\033[26";
        const char REVEAL[] = "\033[27";
        const char CROSSED_OFF[] = "\033[28";
    }
    const char endl[] = "\033[0m\n";

    /**
     * @brief Foreground - text color
     * 
     */
    namespace fg
    {
        const char BLACK[] = "\u001b[30m";
        const char RED[] = "\u001b[31m";
        const char GREEN[] = "\u001b[32m";
        const char YELLOW[] = "\u001b[33m";
        const char BLUE[] = "\u001b[34m";
        const char MAGENTA[] = "\u001b[35m";
        const char CYAN[] = "\u001b[36m";
        const char WHITE[] = "\u001b[37m";
        const char DEFAULT[] = "\u001b[39m";
    }

    /**
     * @brief Bright Foreground - brighter text color
     * 
     */
    namespace bfg
    {
        const char BLACK[] = "\u001b[30;1m";
        const char RED[] = "\u001b[31;1m";
        const char GREEN[] = "\u001b[32;1m";
        const char YELLOW[] = "\u001b[33;1m";
        const char BLUE[] = "\u001b[34;1m";
        const char MAGENTA[] = "\u001b[35;1m";
        const char CYAN[] = "\u001b[36;1m";
        const char WHITE[] = "\u001b[37;1m";
    }

    /**
     * @brief Background - background color
     * 
     */
    namespace bg
    {
        const char BLACK[] = "\u001b[40m";
        const char RED[] = "\u001b[41m";
        const char GREEN[] = "\u001b[42m";
        const char YELLOW[] = "\u001b[43m";
        const char BLUE[] = "\u001b[44m";
        const char MAGENTA[] = "\u001b[45m";
        const char CYAN[] = "\u001b[46m";
        const char WHITE[] = "\u001b[47m";
        const char DEFAULT[] = "\u001b[49m";
    }

    /**
     * @brief Bright Background - brighter background color
     * 
     */
    namespace bbg
    {
        const char BLACK[] = "\u001b[40;1m";
        const char RED[] = "\u001b[41;1m";
        const char GREEN[] = "\u001b[42;1m";
        const char YELLOW[] = "\u001b[43;1m";
        const char BLUE[] = "\u001b[44;1m";
        const char MAGENTA[] = "\u001b[45;1m";
        const char CYAN[] = "\u001b[46;1m";
        const char WHITE[] = "\u001b[47;1m";
    }

    /**
     * @brief Special - framed, encircled, overlined
     * 
     */
    namespace special
    {
        const char FRAMED[] = "\u001b[51m";
        const char ENCIRCLED[] = "\u001b[52m";
        const char OVERLINED[] = "\u001b[53m";
        const char NORMAL1[] = "\u001b[54m";
        const char NORMAL2[] = "\u001b[55m";
    }

    // /**
    //  * Standard background color where n can be a number between 0-7
    //  * High intensity background color where n can be a number between 8-15
    //  * Rainbow background color where n can be a number between 16-231
    //  * Gray background color where n can be a number between 232-255
    //  * 
    //  */
    // const char* get_fg_color(uint8_t n);
}
