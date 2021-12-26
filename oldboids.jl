# initialize screen

WIDTH = 1280
HEIGHT = 720
BACKGROUND = colorant"antiquewhite"

# set number of actors (boids)

n = 200    # 200

# select random starting points

x = rand(20:5:(WIDTH - 20), n)
y = rand(20:5:(HEIGHT - 20), n)

# define initial state of actors

r = 2    # 2

boid = []

for i in 1:n
    push!(boid, Circle(x[i], y[i], r))
end

# select random colors for actors

colors = [colorant"red", colorant"green", colorant"blue"]

boid_color = []

for i in 1:n
    push!(boid_color, rand(colors))
end

# draw actors

function draw(g::Game)
    for i in 1:n
        draw(boid[i], boid_color[i], fill = true)
    end
end

# select random initial velocities

range = [collect(-4:-2); collect(2:4)]

vx = rand(range, n)
vy = rand(range, n)
v = [[vel[1],vel[2]] for vel in zip(vx,vy)]
# initialize acceleration vectors

ax = zeros(n)
ay = zeros(n)

# create steering force vectors for rules

# rx1 = zeros(n)
# ry1 = zeros(n)
# rx2 = zeros(n)
# ry2 = zeros(n)
# rx3 = zeros(n)
# ry3 = zeros(n)
rxy = [[0.0 ,0.0] for i in 1:n]

# define border function

function border(i)
    if boid[i].x > WIDTH
        boid[i].x = 0
    elseif boid[i].x < 0
        boid[i].x = WIDTH
    elseif boid[i].y > HEIGHT
        boid[i].y = 0
    elseif boid[i].y < 0
        boid[i].y = HEIGHT
    end
end

# create distance method

function distance(boid1::Circle, boid2::Circle)
    d = Int(round(sqrt(
        (boid1.x - boid2.x)^2 + (boid1.y - boid2.y)^2
    )))
    return d
end

# set variables for boids

perception_radius = 10    # 10
min_speed = 2             # 2
max_speed = 4             # 4
separation_dial = 60      # 60
alignment_dial = 8        # 8
cohesion_dial = 100       # 100

# define 1 function for all 3 rules
function clip_steering_force(steering_force_x,steering_force_y)
    sfx,sfy = Int(round(steering_force_x)), Int(round(steering_force_y))
    if abs(sfx) < 1
        sfx = Int(round(min_speed * sign(sfx)))
    end
    if abs(sfx) > max_speed
        sfx = Int(round(max_speed * sign(sfx)))
    end

    if abs(sfy) < 1
        sfy = Int(round(min_speed * sign(sfy)))
    end
    if abs(sfy) > max_speed
        sfy = Int(round(max_speed * sign(sfy)))
    end
    return [sfx,sfy]
end
function flock()
    # initialize empty array for separation rule
    # separation_force_x = []
    # separation_force_y = []
    separation_force = []

    # initialize empty arrays for alignment rule
    # neighbor_vx = []
    # neighbor_vy = []
    neighbor_v  =[]
    # initialize empty arrays for cohesion rule
    neighbor_x = []
    neighbor_y = []
    neighbor_xy = []
    # initialize counter
    total = 0

    for i in 1:n
        for j in 1:n
            # search for boids within perception_radius
            d = distance(boid[i], boid[j])
            if boid[i] !== boid[j] && d < perception_radius
                # populate arrays for separation rule
                # sfx = boid[i].x - boid[j].x
                # sfy = boid[i].y - boid[j].y
                # push!(separation_force_x, sfx)
                # push!(separation_force_y, sfy)
                push!(separation_force, [boid[i].x - boid[j].x,boid[i].y - boid[j].y])
                # populate arrays for alignment rule
                # push!(neighbor_vx, vx[j])
                # push!(neighbor_vy, vy[j])
                #push!(neighbor_v, [vx[j],vy[j]])
                push!(neighbor_v,v[j])
                # populate arrays for cohesion rule
                # push!(neighbor_x, boid[j].x)
                # push!(neighbor_y, boid[j].y)
                push!(neighbor_xy,[boid[j].x, boid[j].y])
                # increment counter
                total += 1

                # conditional for boids within perception_radius
                # 1. separation rule #######################################
                avg_xy = sum(separation_force)/total
                sf1 = (avg_xy-v[i])/separation_dial
                rxy[i] = clip_steering_force(sf1[1],sf1[2])

                # 2. alignment rule ########################################rung
                avg_v = sum(neighbor_v)/total
                sf2 = (avg_v -v[i])/alignment_dial
                rxy[i] +=clip_steering_force(sf2[1],sf2[2])

                # 3. cohesion rule #########################################
                avg_xy = sum(neighbor_xy)/total
                sf3 = (avg_xy - [boid[i].x,boid[i].y] - v[i])/cohesion_dial
                rxy[i] += clip_steering_force(sf3[1],sf3[2])


            end
        end
    end
end
function clip_speed(vel)
        v_clipped = vel
        if v_clipped == 0
            v_clipped  = min_speed * rand((-1, 1))
        end
        if abs(v_clipped) > max_speed
            v_clipped  = max_speed * sign(vel)
        end
        return v_clipped
end
# update position of actors (boids)

function update(g::Game)
    global rxy
    flock()
    for i in 1:n
        border(i)
        ax[i],ay[i] = rxy[i]
        v[i] += [ax[i],ay[i]]
        v[i][1],v[i][2]= clip_speed(v[i][1]),clip_speed(v[i][2])
        boid[i].x += v[i][1]
        boid[i].y += v[i][2]
    end
    rxy = [[0.0,0.0] for i in 1:n]
end

#tstopen
