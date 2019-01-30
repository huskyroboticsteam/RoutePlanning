
//
// Disclaimer:
// ----------
//
// This code will work only if you selected window, graphics and audio.
//
// Note that the "Run Script" build phase will copy the required frameworks
// or dylibs to your application bundle so you can execute it on any OS X
// computer.
//
// Your resource files (images, sounds, fonts, ...) are also copied to your
// application bundle. To get the path to these resources, use the helper
// function `resourcePath()` from ResourcePath.hpp
//

#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string>

// Imports resourcePath() for macos
#include "ResourcePath.hpp"

#include "grid.hpp"
#include "Simulator.hpp"
#include "Map.hpp"

#if defined(_WIN32) || defined(__linux__) || defined(__unix__)
    const std::string RESOURCE_DIR = "./Resources/";
    #define WINDOW_SCALE 0.5f
#elif __APPLE__
    const std::string RESOURCE_DIR = resourcePath();
    #define WINDOW_SCALE 1.f
#endif

#if THEME == 0
    const sf::Color bgColor = sf::Color(255, 255, 255);
#else
    const sf::Color bgColor = sf::Color(0, 0, 0);
#endif

int main(int, char const **)
{
    const unsigned int FRAMERATE = 60;

    sf::RenderWindow window(sf::VideoMode(1476 * WINDOW_SCALE, 1576 * WINDOW_SCALE), "Robot Simulator");
    window.setFramerateLimit(FRAMERATE);

    bool hasFocus = true;
    bool robotAuto = false;

    // test bools
    bool vroom = false;
    bool spinny = false;
    bool lazer = false;
    
    sf::Image icon;
    if (icon.loadFromFile(RESOURCE_DIR + "HuskyRoboticsLogo.png"))
    {
        window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());
    }

    sf::Clock clock;
    static const sf::Time AUTO_INTERVAL = sf::seconds(0.5);

    Grid grid(40.f, 40.f, 36 * WINDOW_SCALE);
    float gridScale = grid.retrieveScale();
    float gridHeight = grid.retrieveHeight();
    Agent agent(gridScale, grid.retrieveWidth(), gridHeight, 2.f, 2.f);
    grid.target = RP::point{35.f, 35.f};
    RP::Simulator sim(grid.obstacleList, agent, RP::simulator_config{70.f, 10.f}, gridScale, gridHeight);
    RP::Map map(sim.getpos(), grid.target);
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            // Close window on X or Cmd+W
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
            else if (event.type == sf::Event::GainedFocus)
            {
                hasFocus = true;
            }
            else if (event.type == sf::Event::LostFocus)
            {
                hasFocus = false;
            }
            else if (event.type == sf::Event::KeyPressed && hasFocus) {
                switch (event.key.code) {
                    case sf::Keyboard::H : {
                        std::cout << "Help Menu: " << std::endl;
                        std::cout << "W/S -- Drive robot forward or back" << std::endl;
                        std::cout << "A/D -- Rotate robot left or right" << std::endl;
                        std::cout << "P   -- Returns the internal position of the robot" << std::endl;
                        std::cout << "G   -- Toggle grid" << std::endl;
                        std::cout << "O   -- Import obstacles from obstacles.txt" << std::endl;
                        std::cout << "N   -- Toggle clipping" << std::endl;
                        std::cout << "0   -- Clear robot path" << std::endl;
                        std::cout << "7   -- Drive forward at max speed" << std::endl;
                        std::cout << "8   -- Turn towards the target" << std::endl;
                        std::cout << "9   -- Draw algorithm path" << std::endl;
                        std::cout << "Up/Down    -- Teleport robot 0.5m forward or back" << std::endl;
                        std::cout << "Left/Right -- Teleport robot 15 degrees left or right" << std::endl;
                        break;
                    }
                    case sf::Keyboard::P : {
                        std::cout << "Internal Position: (" << agent.getX() << "," << agent.getY()
                                  << ") at " << agent.getInternalRotation() << " degrees" << std::endl;
                        break;
                    }
                    case sf::Keyboard::G : {
                        grid.toggleGrid();
                        break;
                    }
                    case sf::Keyboard::O : {
                        grid.obstacleList.clear();
                        grid.readObstaclesFromFile(RESOURCE_DIR + "obstacles.txt");
                        grid.addBorderObstacles();
                        std::cout << "Added obstacles" << std::endl;
                        break;
                    }
                    case sf::Keyboard::N : {
                        grid.toggleClipping();
                        break;
                    }
                    case sf::Keyboard::Tilde : {
                        robotAuto = !robotAuto;
                        break;
                    }
                    case sf::Keyboard::Num7 : {
                        vroom = !vroom;
                        break;
                    }
                    case sf::Keyboard::Num8 : {
                        spinny = !spinny;
                        break;
                    }
                    case sf::Keyboard::Num9 : {
                        if (lazer)
                            grid.drawPath();
                        lazer = !lazer;
                        break;
                    }
                    case sf::Keyboard::Num0 : {
                        agent.clearPath();
                        break;
                    }
                    case sf::Keyboard::Up : {
                        grid.moveAgent(agent, .5f);
                        break;
                    }
                    case sf::Keyboard::Down : {
                        grid.moveAgent(agent, -.5f);
                        break;
                    }
                    case sf::Keyboard::Left : {
                        grid.rotateAgent(agent, 15.f);
                        break;
                    }
                    case sf::Keyboard::Right : {
                        grid.rotateAgent(agent, -15.f);
                        break;
                    }
                }
            }
        }

        if (hasFocus)
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
            {
                grid.moveAgent(agent, 10.f / FRAMERATE);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
            {
                grid.moveAgent(agent, -10.f / FRAMERATE);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
                grid.rotateAgent(agent, 200.f / FRAMERATE);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
                grid.rotateAgent(agent, -200.f / FRAMERATE);
            }
        }
        
        if (vroom)
            grid.moveAgent(agent, agent.drive());
        if (spinny) {
            RP::point st_target = map.compute_next_point();
            grid.rotateAgent(agent, agent.turnTowards(st_target.x, st_target.y));
        }
        if (lazer)
            grid.drawPath(map.shortest_path_to(), agent);
            
        sim.update_agent();
        
        window.clear(bgColor);
        window.draw(grid);
        window.draw(agent);
        window.draw(sim);
        // printf("%f, %f\n", next.x, next.y);
        window.display();
    }

    return EXIT_SUCCESS;
}
