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
v = [[ve[1],ve[2]] for ve in zip(vx,vy)]

# initialize acceleration vectors

ax = zeros(n)
ay = zeros(n)

# create steering force vectors for rules

rx1 = zeros(n)
ry1 = zeros(n)
rx2 = zeros(n)
ry2 = zeros(n)
rx3 = zeros(n)
ry3 = zeros(n)
rxy1 = [[x[1],x[2]] for x in zip(zeros(n),zeros(n))]
rxy2 = [[x[1],x[2]] for x in zip(zeros(n),zeros(n))]
rxy3 = [[x[1],x[2]] for x in zip(zeros(n),zeros(n))]
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

function clip_steering_force(steering_force)
    sfx = steering_force[1]
    sfy= steering_force[2]
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
    #separation_force_x = []
    # separation_force_y = []
    separation_force =[]
    # initialize empty arrays for alignment rule
    # neighbor_vx = []
    # neighbor_vy = []
    neighbor_v = []
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
                # push!(separation_force_x, boid[i].x - boid[j].x)
                # push!(separation_force_y, boid[i].y - boid[j].y)
                push!(separation_force, [boid[i].x - boid[j].x,boid[i].y - boid[j].y])
                # populate arrays for alignment rule
                # push!(neighbor_vx, vx[j])
                # push!(neighbor_vy, vy[j])
                push!(neighbor_v,[vx[j],vy[j]])
                # populate arrays for cohesion rule
                push!(neighbor_x, boid[j].x)
                push!(neighbor_y, boid[j].y)
                push!(neighbor_xy, [boid[j].x,boid[j].y])
                # increment counter
                total += 1

                # conditional for boids within perception_radius
                if total > 0

                    # 1. separation rule #######################################
                    avesepforce = sum(separation_force)/total
                    rxy1[i] = clip_steering_force((avesepforce-v[i])/separation_dial)


                    # 2. alignment rule ########################################
                    avg_v = sum(neighbor_v)/total
                    sf2 = clip_steering_force((avg_v - v[i])/alignment_dial)
                    rx2[i] = Int(round(sf2[1]))
                    ry2[i] = Int(round(sf2[2]))

                    # 3. cohesion rule #########################################
                    avg_xy = sum(neighbor_xy)/total
                    boid_pos = [boid[i].x,boid[i].y]
                    sf3 = clip_steering_force((avg_xy - boid_pos -v[i])/cohesion_dial)
                    rx3[i] = Int(round(sf3[1]))
                    ry3[i] = Int(round(sf3[2]))

                end
            end
        end
    end
end

# update position of actors (boids)

function update(g::Game)
    global rx1, ry1, rx2, ry2, rx3, ry3,rxy1,rxy2,rxy3
    flock()

    for i in 1:n
        border(i)

        # calculate accelerations
        ax[i] = rx1[i] + rx2[i] + rx3[i]
        ay[i] = ry1[i] + ry2[i] + ry3[i]

        # update velocities
        vx[i] += ax[i]
        vy[i] += ay[i]

        if vx[i] == 0
            vx[i] = min_speed * rand((-1, 1))
        end
        if vy[i] == 0
            vy[i] = min_speed * rand((-1, 1))
        end

        if abs(vx[i]) > max_speed
            vx[i] = max_speed * sign(vx[i])
        end
        if abs(vy[i]) > max_speed
            vy[i] = max_speed * sign(vy[i])
        end

        # update positions
        boid[i].x += vx[i]
        boid[i].y += vy[i]

    end

    # clear steering force vectors

    rxy1 = [[x[1],x[2]] for x in zip(zeros(n),zeros(n))]
    rxy2 = [[x[1],x[2]] for x in zip(zeros(n),zeros(n))]
    rxy3 = [[x[1],x[2]] for x in zip(zeros(n),zeros(n))]
end
