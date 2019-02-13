#include "agent.hpp"

Agent::Agent(unsigned int mapScale, float mapW, float mapH, float startX, float startY, float startR, float tSpeed, float rSpeed)
{
    // internal position stored in meters
    xPos = startX;
    yPos = startY;
    // internal rotation stored in degrees
    rotation = startR;
    // internal speed that is currently unused
    transSpeed = tSpeed;
    rotSpeed = rSpeed;

    // the map's scale (pixels per meter)
    scale = mapScale;
    mapWidth = mapW;
    mapHeight = mapH;

    // default agent size in pixels (meters * pixels per meter)
    float agentWidth = 1.f * scale;
    float agentLength = 1.5f * scale;

    // this is a rectangle
    shapeBase = sf::RectangleShape(sf::Vector2f(agentWidth, agentLength));
    shapeBase.setOrigin(agentWidth / 2.f, agentLength / 2.f);
    shapeBase.setFillColor(BASE_COLOR);
    shapeBase.setPosition((xPos + 1) * scale, (mapHeight - yPos) * scale);
    shapeBase.setRotation(90 + startR);

    // this is a triangle, drawn on top of the rectangle
    shapeTop = sf::CircleShape(agentWidth / 2.f, 3);
    shapeTop.setOrigin(agentWidth / 2.f, agentWidth / 2.f);
    shapeTop.setFillColor(TOP_COLOR);
    shapeTop.setPosition((xPos + 1) * scale, (mapHeight - yPos) * scale);
    shapeTop.setRotation(90 + startR);

    // the hitbox is stored as four vectors in polar coordinates
    // each vector corresponds to the displacement of one vertex from the center of the agent
    hitBox[0] = {.9f, atan2(.75f, -.5f)};
    hitBox[1] = {.9f, atan2(.75f, .5f)};
    hitBox[2] = {.9f, atan2(-.75f, .5f)};
    hitBox[3] = {.9f, atan2(-.75f, -.5f)};

    // draws the path the agent has taken
    path.setPrimitiveType(sf::Lines);
}

// simply translates the agent by a certain x and y, regardless of orientation
// adds the movement to the drawn path
void Agent::move(float dx, float dy)
{
    xPos += dx;
    yPos += dy;

    path.append(shapeBase.getPosition());

    // SFML y position (top is 0) is opposite of internal y position (bottom is 0)
    shapeBase.move(dx * scale, -dy * scale);
    shapeTop.move(dx * scale, -dy * scale);

    path.append(shapeBase.getPosition());

    path[path.getVertexCount() - 2].color = PATH_COLOR;
    path[path.getVertexCount() - 1].color = PATH_COLOR;
}

float Agent::rotateTowards(float x, float y)
{
    float tr = atan2(y - yPos, x - xPos) * 180 / PI;
    
    //rotate(tr - rotation);
    return tr - rotation;
}

float Agent::drive(float speed) {
    float dx = transSpeed * speed * cos(rotation * PI / 180);
    float dy = transSpeed * speed * sin(rotation * PI / 180);
    
    //move(dx, dy);
    return transSpeed * speed; // ds
}

float Agent::driveTowards(float targetX, float targetY) {
    // TODO
    return 0.f;
}

float Agent::turn(float speed) {
    if (fabs(speed) > 1)
        speed = speed / fabs(speed);
    
    //rotate(-rotSpeed * speed);
    return -rotSpeed * speed;
}

float Agent::turnTowards(float targetAngle) {
    float rDiff = targetAngle - rotation;
    
    float speed = 1.f;
    
    if (fabs(rDiff) < 60)
        speed = fabs(rDiff) / 60.f;
    if (fabs(rDiff) < 5)
        speed = 5.f / 60.f;
    
    if (rDiff < -180 || (rDiff > 0 && rDiff < 180))
        return turn(-speed);
    else
        return turn(speed);
}

float Agent::turnTowards(float targetX, float targetY) {
    float targetR = atan2(targetY - yPos, targetX - xPos) * 180 / PI;
    if (targetR < 0)
        targetR += 360;
    
    return turnTowards(targetR);
}

// simply rotates the agent by a certain angle
void Agent::rotate(float dr)
{
    rotation += dr;
    
    if (rotation > 360)
        rotation -= 360;
    else if (rotation < 0)
        rotation += 360;

    // SFML rotation (clockwise) is opposite of internal rotation (counter-clockwise)
    shapeBase.rotate(-dr);
    shapeTop.rotate(-dr);
}

// erases the path drawn so far
void Agent::clearPath() { path.clear(); }

void Agent::draw(sf::RenderTarget &target, sf::RenderStates states) const
{
    states.transform *= getTransform();

    target.draw(path, states);
    target.draw(shapeBase, states);
    target.draw(shapeTop, states);
}
