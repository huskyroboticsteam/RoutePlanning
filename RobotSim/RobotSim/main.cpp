
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
#include <queue>

// Imports resourcePath() for macos
#include "ResourcePath.hpp"
#include "grid.hpp"
#include "Simulator.hpp"
#include "simController.hpp"
#include "ui.hpp"

#if defined(_WIN32) || defined(__linux__) || defined(__unix__)
const std::string RESOURCE_DIR = "./Resources/";
#define WINDOW_SCALE 1.f
#elif __APPLE__
const std::string RESOURCE_DIR = resourcePath();
#define WINDOW_SCALE 1.f
#endif

#if THEME == 0
const sf::Color bgColor = sf::Color(255, 255, 255);
#else
const sf::Color bgColor = sf::Color(0, 0, 0);
#endif

constexpr float RECOMPUTE_COOLDOWN = 1.5f; // time between recomputation of the path/graph

static inline sf::CircleShape getNode(RP::node, float scale, float height);

void draw_qtree(sf::RenderWindow &win, const RP::QTreeNode& node, float scale, float height);

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

    // autonomous
    bool auton = false;
    bool auto_turning = false;
    float auto_orig_angle; // angle of robot before it entered turning phase

    // states
    bool showGraph = false;
    RP::Timer recompute_timer;

    sf::Image icon;
    if (icon.loadFromFile(RESOURCE_DIR + "HuskyRoboticsLogo.png"))
    {
        window.setIcon(icon.getSize().x, icon.getSize().y, icon.getPixelsPtr());
    }

    Grid grid(40.f, 40.f, 36 * WINDOW_SCALE);
    float gridScale = grid.retrieveScale();
    float gridHeight = grid.retrieveHeight();
    Agent agent(gridScale, grid.retrieveWidth(), gridHeight, 2.5f, 2.5f, 45.f);
    agent.bot_width = 1.8f;
    grid.target = RP::point{35.f, 35.f};
    RP::Simulator sim(grid.obstacleList, agent, RP::simulator_config{70.f, 10.f}, gridScale, gridHeight);
    RP::Pather pather(sim.getpos(), grid.target, RP::point{39.f, 39.f}, agent.bot_width);

    RP::SimController control(grid, agent, pather);

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
            else if (event.type == sf::Event::KeyPressed && hasFocus)
            {
                switch (event.key.code)
                {
                case sf::Keyboard::H:
                {
                    std::cout << "Help Menu: " << std::endl;
                    std::cout << "W/S -- Drive robot forward or back" << std::endl;
                    std::cout << "A/D -- Rotate robot left or right" << std::endl;
                    std::cout << "P   -- Returns the internal position of the robot" << std::endl;
                    std::cout << "G   -- Toggle grid" << std::endl;
                    std::cout << "O   -- Import obstacles from obstacles.txt" << std::endl;
                    std::cout << "U   -- Complete autonomous mode" << std::endl;
                    std::cout << "N   -- Toggle clipping" << std::endl;
                    std::cout << "E   -- Toggle show graph" << std::endl;
                    std::cout << "0   -- Clear robot path" << std::endl;
                    std::cout << "7   -- Drive forward at max speed" << std::endl;
                    std::cout << "8   -- Turn towards the target" << std::endl;
                    std::cout << "9   -- Draw algorithm path" << std::endl;
                    std::cout << "Up/Down    -- Teleport robot 0.5m forward or back" << std::endl;
                    std::cout << "Left/Right -- Teleport robot 15 degrees left or right" << std::endl;
                   break;
                }
                case sf::Keyboard::P:
                {
                    std::cout << "Internal Position: (" << agent.getX() << "," << agent.getY()
                              << ") at " << agent.getInternalRotation() << " degrees" << std::endl;
                    break;
                }
                case sf::Keyboard::G:
                {
                    grid.toggleGrid();
                    break;
                }
                case sf::Keyboard::O:
                {
                    grid.obstacleList.clear();
                    grid.readObstaclesFromFile(RESOURCE_DIR + "obstacles.txt");
                    // grid.addBorderObstacles();
                    std::cout << "Added obstacles" << std::endl;
                    break;
                }
                case sf::Keyboard::U:
                {
                    auton = !auton;
                    if (auton)
                        control.start_auto();
                    else
                        control.stop_auto();
                    break;
                }
                case sf::Keyboard::N:
                {
                    grid.toggleClipping();
                    break;
                }
                case sf::Keyboard::E:
                {
                    showGraph = !showGraph;
                    break;
                }
                case sf::Keyboard::Tilde:
                {
                    robotAuto = !robotAuto;
                    break;
                }
                case sf::Keyboard::Num7:
                {
                    vroom = !vroom;
                    break;
                }
                case sf::Keyboard::Num8:
                {
                    spinny = !spinny;
                    break;
                }
                case sf::Keyboard::Num9:
                {
                    if (lazer)
                        grid.drawPath();
                    lazer = !lazer;
                    break;
                }
                case sf::Keyboard::Num0:
                {
                    agent.clearPath();
                    break;
                }
                case sf::Keyboard::Up:
                {
                    grid.moveAgent(agent, .5f);
                    break;
                }
                case sf::Keyboard::Down:
                {
                    grid.moveAgent(agent, -.5f);
                    break;
                }
                case sf::Keyboard::Left:
                {
                    grid.rotateAgent(agent, 15.f);
                    break;
                }
                case sf::Keyboard::Right:
                {
                    grid.rotateAgent(agent, -15.f);
                    break;
                }
                    // case sf::Keyboard::J:
                    // {
                    //     for (const auto& o : sim.visible_obstacles())
                    //         printf("(%f, %f), (%f, %f)\n", o.coord1.x, o.coord1.y, o.coord2.x, o.coord2.y);
                    // }
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
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
            {
                grid.rotateAgent(agent, 200.f / FRAMERATE);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
            {
                grid.rotateAgent(agent, -200.f / FRAMERATE);
            }
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
        if (auton)
        {
            control.tic();
            if (lazer)
                grid.drawPath(pather.get_cur_path(), agent);
        }
        else
        {
            if (vroom)
            {
                //grid.moveAgent(agent, agent.drive());
                RP::point st_target = pather.get_cur_next_point();
                if (st_target.x != INFINITY)
                    grid.moveAgent(agent, agent.driveTowards(st_target.x, st_target.y));
            }

            if (spinny && lazer)
            {
                auto path = pather.get_cur_path();
                grid.rotateAgent(agent, agent.turnTowards(path.front().x, path.front().y));
                grid.drawPath(path, agent);
            }
            else
            {
                if (spinny)
                {
                    RP::point st_target = pather.get_cur_next_point();
                    if (st_target.x != INFINITY)
                        grid.rotateAgent(agent, agent.turnTowards(st_target.x, st_target.y));
                }
                if (lazer)
                    grid.drawPath(pather.get_cur_path(), agent);
            }
        }
        sim.update_agent();
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
