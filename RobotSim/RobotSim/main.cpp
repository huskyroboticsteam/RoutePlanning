
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
#include <cmath>
#include <queue>

// Imports resourcePath() for macos
#include "ResourcePath.hpp"
#include "grid.hpp"
#include "Simulator.hpp"
#include "WorldCommunicator.hpp"
#include "simController.hpp"
#include "ui.hpp"

#if defined(_WIN32) || defined(__linux__) || defined(__unix__)
const std::string RESOURCE_DIR = "./Resources/";
#define WINDOW_SCALE .5f
#elif __APPLE__
const std::string RESOURCE_DIR = resourcePath();
#define WINDOW_SCALE 1.f
#endif

// ---------------------------------------- //
// ---------- Internal Variables ---------- //
// ---------------------------------------- //
const sf::Color bgColor = sf::Color(255, 255, 255);

constexpr float RECOMPUTE_COOLDOWN = 1.5f; // time between recomputation of the path/graph

static inline sf::CircleShape getNode(RP::node, float scale, float height);

void draw_qtree(sf::RenderWindow &win, const RP::QTreeNode& node, float scale, float height);

Grid grid(40.f, 40.f, 36 * WINDOW_SCALE);
float gridScale = grid.retrieveScale();
float gridWidth = grid.retrieveWidth();
float gridHeight = grid.retrieveHeight();
Agent agent(gridScale, gridWidth, gridHeight, RP::point{2.5f, 2.5f}, 45.f);

RP::Simulator sim(grid.obstacleList, agent, RP::simulator_config{70.f, 10.f}, gridScale, gridHeight);
RP::Pather pather(sim.getpos(), grid.target, RP::point{39.f, 39.f}, agent.bot_width);

RP::SimController control(grid, agent, pather);

WorldCommunicator worldCommunicator;
static float goalDirection;
static float toMove;
// ---------------------------------------- //


// ---------------------------------------- //
// ---------- BeagleBone Methods ---------- //
// ---------------------------------------- //

// moves the robot forward or backward at a given speed
// speed should be between 1 and -1, where negative is backwards
void move(float speed) {
    grid.moveAgent(agent, agent.drive(speed));
}

// rotates the robot clockwise or counterclockwise at a given speed
// speed should be between 1 and -1, where negative is counterclockwise
void turn(float speed) {
    grid.rotateAgent(agent, agent.turn(speed));
}

void turnTo(float goalDirection) {
	//std::cout << "goal direction: " << goalDirection << std::endl;
	//std::cout << "internal direction: " << agent.getInternalRotation() << std::endl;
	if(abs(goalDirection - agent.getInternalRotation()) > 1.0) {
		if(goalDirection > agent.getInternalRotation()) {
			turn(-1);
		} else {
			turn(1);
		}
	} else {
		turn(0);
	}
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
// ---------------------------------------- //


int main(int, char const **)
{
    sf::RenderWindow window(sf::VideoMode(1476 * WINDOW_SCALE, 1576 * WINDOW_SCALE), "Robot Simulator");
    window.setFramerateLimit(60);
    
    // ADDITIONAL SETUP FROM INIT
    agent.bot_width = 1.8f;
    grid.target = RP::point{35.f, 35.f};
    // END ADDITIONAL SETUP
    
    // states
    bool showGraph = false;
    RP::Timer recompute_timer;

    sf::Image icon;
    if (icon.loadFromFile(RESOURCE_DIR + "HuskyRoboticsLogo.png"))
    {
        window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());
    }
    
    // setting toggles
    bool lazer = false;
    bool breadcrumb = false;

    // autonomous
    bool auton = false;
    bool auto_turning = false;
    float auto_orig_angle; // angle of robot before it entered turning phase

    
    // OPTIONAL FURTHER INIT
    //agent.scaleSpeed(2.f);
    grid.target = RP::point{35.f, 35.f};
    // END INIT

    // ---------------------------------------- //
    // ---------- 60 FPS Update Loop ---------- //
    // ---------------------------------------- //
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
                        std::cout << "E   -- Show pathing graph" << std::endl;
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
                    case sf::Keyboard::E : {
                        showGraph = !showGraph;
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
        
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
            // move forwards
            move(1);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
            // move backwards
            move(-1);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
            // turn left
            turn(-1);
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
            // turn right
            turn(1);
        }
        
        window.clear(bgColor);
        const RP::graph &dg = pather.d_graph();
        // draw nodes
        if (showGraph && !dg.nodes.empty())
        {
            std::vector<bool> visited(dg.nodes.size(), false);
            std::queue<int> q;
            q.push(0);
            while (!q.empty())
            {
                int ind = q.front();
                const auto &nd = dg.nodes[ind];
                q.pop();
                visited[ind] = true;
                for (const RP::eptr edge : nd.connection)
                {
                    if (!visited[edge->child])
                    {
                        q.push(edge->child);
                        window.draw(get_vertex_line(nd.coord, dg.nodes[edge->child].coord, GRAPH_EDGE_COLOR, gridScale, gridHeight));
                    }
                }
                if (ind != 0)
                    window.draw(getNode(nd, gridScale, gridHeight));
            }
        }
        
        if (lazer)
            grid.drawPath(pather.get_cur_path(), agent);
        if (auton)
            control.tic();
        
        sim.update_agent();
		
        
		float change = 0.0;
		worldCommunicator.update(currentPosition(), currentRotation(), change, toMove);
		goalDirection += change;
		goalDirection = (int)goalDirection%360;
		//turnTo(goalDirection);
		//move(toMove);
		
        
        pather.set_pos(sim.getpos());
        pather.add_obstacles(sim.visible_obstacles());
        if (!auton && recompute_timer.elapsed() > RECOMPUTE_COOLDOWN)
        {
            recompute_timer.reset();
            //pather.compute_path();
        }
        const RP::QTreeNode& root = *pather.debug_qtree_root();
        draw_qtree(window, root, gridScale, gridHeight);

        
        window.draw(grid);
        window.draw(agent);
        for (auto obst : pather.mem_obstacles())
            window.draw(get_vertex_line(obst.p, obst.q, SEEN_OBST_COLOR, gridScale, gridHeight));
        window.draw(sim);
        // printf("%f, %f\n", next.x, next.y);

        window.display();
    }
    // ---------- End of 60 FPS Update Loop ---------- //

    return EXIT_SUCCESS;
}

void draw_qtree(sf::RenderWindow &win, const RP::QTreeNode& node, float scale, float height)
{
    sf::RectangleShape rect;
    rect.setSize(sf::Vector2f((node.max_x - node.min_x) * scale,
                              (node.max_y - node.min_y) * scale));
    rect.setOutlineColor(sf::Color(0, 255, 255));
    rect.setOutlineThickness(1.f);
    rect.setPosition((node.min_x + 1) * scale, (height - node.max_y) * scale);
    sf::Color fillColor = node.is_blocked ? sf::Color(0, 255, 255, 64) : sf::Color(0, 0, 0, 0);
    rect.setFillColor(fillColor);
    win.draw(rect);
    if (!node.is_leaf)
    {
        draw_qtree(win, *node.topleft, scale, height);
        draw_qtree(win, *node.topright, scale, height);
        draw_qtree(win, *node.botleft, scale, height);
        draw_qtree(win, *node.botright, scale, height);
    }
}

static inline sf::CircleShape getNode(RP::node nd, float scale, float height)
{
    sf::CircleShape circle(5);
    circle.setOrigin(5, 5);
    circle.setFillColor(GRAPH_NODE_COLOR);
    circle.setPosition((nd.coord.x + 1) * scale, (height - nd.coord.y) * scale);
    return circle;
}
