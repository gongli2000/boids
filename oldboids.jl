# initialize screen
using GameZero,Colors
mutable struct Boid
    pos::Vector{Int64}
    v::Vector{Int64}
    color::RGB
end

# set number of actors (boids)

n = 200    # 200

# select random starting points

# x = rand(20:5:(WIDTH - 20), n)
# y = rand(20:5:(HEIGHT - 20), n)

# define initial state of actors

r = 2    # 2


# vx = rand(xrange, n)
# vy = rand(xrange, n)
WIDTH = 1280
HEIGHT = 720
BACKGROUND = colorant"antiquewhite"
xrange = [collect(-4:-2); collect(2:4)]
colors = [colorant"red", colorant"green", colorant"blue"]
boid = [Boid([rand(20:5:(WIDTH - 20)),rand(20:5:(HEIGHT - 20))]
                ,[rand(xrange),rand(xrange)]
                ,rand(colors))
         for i in 1:n]

# for i in 1:n
#     push!(boid, Circle(x[i], y[i], r))
# end

# select random colors for actors



# boid_color = []
#
# for i in 1:n
#     push!(boid_color, rand(colors))
# end

# draw actors

function draw(g::Game)
    for i in 1:n
        draw(Circle(boid[i].pos[1],boid[i].pos[2],r), boid[i].color, fill = true)
    end
end

# select random initial velocities




# create steering force vectors for rules

# rx1 = zeros(n)
# ry1 = zeros(n)
# rx2 = zeros(n)
# ry2 = zeros(n)
# rx3 = zeros(n)
# ry3 = zeros(n)
rxy = [[0.0 ,0.0] for i in 1:n]

# define border function

function clip_pos(pos::Vector)
    xpos = pos[1]
    ypos = pos[2]
    if xpos> WIDTH
        xpos = 0
    elseif xpos < 0
        xpos = WIDTH
    elseif ypos > HEIGHT
        ypos = 0
    elseif ypos < 0
        ypos = HEIGHT
    end
    return [xpos,ypos]
end

# create distance method

function distance(boid1::Boid, boid2::Boid)
    dp =boid1.pos - boid2.pos
    return Int(round(sqrt(sum(dp .* dp))))
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
    separation_force = []

    # initialize empty arrays for alignment rule


    total = 0
    neighbor_v  =[]
    neighbor_xy = []
    for i in 1:n

        for j in 1:n
            # search for boids within perception_radius
            d = distance(boid[i], boid[j])
            if boid[i] !== boid[j] && d < perception_radius
                total += 1
                push!(separation_force,boid[i].pos-boid[j].pos) #separation rule
                push!(neighbor_v,boid[j].v) # alignment rule
                push!(neighbor_xy,boid[j].pos) # cohesion rule

                # conditional for boids within perception_radius
                # 1. separation rule #######################################
                avg_xy = sum(separation_force)/total
                sf1 = (avg_xy-boid[i].v)/separation_dial
                rxy[i] = clip_steering_force(sf1[1],sf1[2])

                # 2. alignment rule ########################################rung
                avg_v = sum(neighbor_v)/total
                sf2 = (avg_v -boid[i].v)/alignment_dial
                rxy[i] += clip_steering_force(sf2[1],sf2[2])

                # 3. cohesion rule #########################################
                avg_xy = sum(neighbor_xy)/total
                sf3 = (avg_xy - boid[i].pos - boid[i].v)/cohesion_dial
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
        boid[i].pos = clip_pos(boid[i].pos)
        boid[i].v += rxy[i]
        boid[i].v= [clip_speed(boid[i].v[1]),clip_speed(boid[i].v[2])]
        boid[i].pos += boid[i].v
    end
    rxy = [[0.0,0.0] for i in 1:n]
end

#tstopen
