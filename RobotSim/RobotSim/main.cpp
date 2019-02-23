
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

Grid grid(40.f, 40.f, 36 * WINDOW_SCALE);
float gridScale = grid.retrieveScale();
float gridHeight = grid.retrieveHeight();
Agent agent(gridScale, grid.retrieveWidth(), gridHeight, RP::point{2, 2});
RP::Simulator sim(grid.obstacleList, agent, RP::simulator_config{70.f, 10.f}, gridScale, gridHeight);
RP::Map map(sim.getpos(), grid.target);

RP::AutoController control(grid, agent, map);

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
    
    // OPTIONAL FURTHER INIT
    agent.scaleSpeed(2.f);
    grid.target = RP::point{35.f, 35.f};
    // END INIT

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
                    default : {
                        std::cout << "Command not recognized" << std::endl;
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

// moves the robot forward or backward at a given speed
// speed should be between 1 and -1, where negative is backwards
static void move(float speed) {
    grid.moveAgent(agent, speed);
}

// rotates the robot clockwise or counterclockwise at a given speed
// speed should be between 1 and -1, where negative is clockwise
static void turn(float speed) {
    grid.rotateAgent(agent, speed);
}

// returns the current position of the robot, in meters, relative to the grid origin
// x and y increase to the right and to the top respectively
static RP::point currentPosition() {
    return RP::point{ agent.getX(), agent.getY() };
}

// returns the current rotation of the robot, in degrees increasing counterclockwise
// value is between 0 and 360, where 0 is true right relative to the grid
static float currentRotation() {
    return agent.getInternalRotation();
}

// returns all the obstacles currently visible to the robot
// note that these may be partial obstacles
// TODO may return an RP::obstacle depending on implementation
static std::vector<RP::line> currentObstaclesInView() {
    // TODO
}
