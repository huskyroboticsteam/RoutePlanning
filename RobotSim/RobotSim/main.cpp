
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
#include "autoController.hpp"

#if defined(_WIN32) || defined(__linux__) || defined(__unix__)
const std::string RESOURCE_DIR = "./Resources/";
#define WINDOW_SCALE 0.5f
#elif __APPLE__
const std::string RESOURCE_DIR = resourcePath();
#define WINDOW_SCALE 1.f
#endif

const sf::Color bgColor = sf::Color(255, 255, 255);

int main(int, char const **)
{
    const unsigned int FRAMERATE = 60;

    sf::RenderWindow window(sf::VideoMode(1476 * WINDOW_SCALE, 1576 * WINDOW_SCALE), "Robot Simulator");
    window.setFramerateLimit(FRAMERATE);

    // setting toggles
    bool lazer = false;
    bool breadcrumb = false;

    // autonomous
    bool auton = false;
    bool auto_turning = false;
    float auto_orig_angle; // angle of robot before it entered turning phase

    sf::Image icon;
    if (icon.loadFromFile(RESOURCE_DIR + "HuskyRoboticsLogo.png"))
    {
        window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());
    }

    Grid grid(40.f, 40.f, 36 * WINDOW_SCALE);
    float gridScale = grid.retrieveScale();
    float gridHeight = grid.retrieveHeight();
    Agent agent(gridScale, grid.retrieveWidth(), gridHeight, 2.f, 2.f);
    agent.scaleSpeed(2.f);
    grid.target = RP::point{35.f, 35.f};
    RP::Simulator sim(grid.obstacleList, agent, RP::simulator_config{70.f, 10.f}, gridScale, gridHeight);
    RP::Map map(sim.getpos(), grid.target);

    RP::AutoController control(grid, agent, map);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            // Close window on X or Cmd+W
            if (event.type == sf::Event::Closed)
                window.close();
            
            else if (event.type == sf::Event::KeyPressed)
            {
                switch (event.key.code) {
                case sf::Keyboard::H : {
                    std::cout << "Help Menu: " << std::endl;
                    std::cout << "P   -- Returns the internal position of the robot" << std::endl;
                    std::cout << "G   -- Toggle grid" << std::endl;
                    std::cout << "O   -- Import obstacles from obstacles.txt" << std::endl;
                    std::cout << "U   -- Complete autonomous mode" << std::endl;
                    std::cout << "N   -- Toggle clipping" << std::endl;
                    std::cout << "0   -- Toggle robot path" << std::endl;
                    std::cout << "9   -- Draw algorithm path" << std::endl;
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
                    // grid.addBorderObstacles();
                    std::cout << "Added obstacles" << std::endl;
                    break;
                }
                case sf::Keyboard::U : {
                    auton = !auton;
                    if (auton)
                        control.start_auto();
                    else
                        control.stop_auto();
                    break;
                }
                case sf::Keyboard::N : {
                    grid.toggleClipping();
                    break;
                }
                case sf::Keyboard::Num9 : {
                    if (lazer)
                        grid.drawPath();
                    lazer = !lazer;
                    break;
                }
                case sf::Keyboard::Num0 : {
                    agent.togglePath();
                    break;
                }
                case sf::Keyboard::B : {
                    map.breakpoint(); // temp; used for debug only
                }
                }
            }
        }
        
        if (auton)
            control.tic();
        if (lazer)
            grid.drawPath(map.shortest_path_to(), agent);
        
        sim.update_agent();
        map.update(sim.visible_obstacles());

        window.clear(bgColor);
        window.draw(grid);
        window.draw(agent);
        for (auto obst : map.memo_obstacles())
            window.draw(get_vertex_line(obst.coord1, obst.coord2, SEEN_OBST_COLOR, gridScale, gridHeight));
        window.draw(sim);
        // printf("%f, %f\n", next.x, next.y);
        window.display();
    }

    return EXIT_SUCCESS;
}

static void move(float x, float y);
static void turn(float rotation);
static RP::point currentPosition();
static float currentRotation();
static std::vector<RP::line> currentObstaclesInView();
