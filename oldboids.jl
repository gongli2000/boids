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
    sfx,sfy = steering_force_x, steering_force_y
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
    return sfx,sfy
end
function flock()
    # initialize empty array for separation rule
    # separation_force_x = []
    # separation_force_y = []
    separation_force = []

    # initialize empty arrays for alignment rule
    neighbor_vx = []
    neighbor_vy = []
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
                push!(neighbor_vx, vx[j])
                push!(neighbor_vy, vy[j])
                # populate arrays for cohesion rule
                push!(neighbor_x, boid[j].x)
                push!(neighbor_y, boid[j].y)
                # increment counter
                total += 1

                # conditional for boids within perception_radius
                if total > 0

                    # 1. separation rule #######################################
                    avg_xy = sum(separation_force)/total
                    avg_x1 = Int(round(avg_xy[1]))
                    avg_y1 = Int(round(avg_xy[2]))


                    steering_force_x1 = Int(round(
                        (avg_x1 - vx[i]) / separation_dial
                    ))
                    steering_force_y1 = Int(round(
                        (avg_y1 - vy[i]) / separation_dial
                    ))
                    rx1[i],ry1[i] = clip_steering_force(steering_force_x1,steering_force_y1)


                    # 2. alignment rule ########################################
                    avg_vx = Int(round(
                        sum(neighbor_vx) / total
                    ))
                    avg_vy = Int(round(
                        sum(neighbor_vy) / total
                    ))

                    steering_force_x2 = Int(round(
                        (avg_vx - vx[i]) / alignment_dial
                    ))
                    steering_force_y2 = Int(round(
                        (avg_vy - vy[i]) / alignment_dial
                    ))

                    rx2[i],ry2[i]=clip_steering_force(steering_force_x2,steering_force_y2)


                    # 3. cohesion rule #########################################
                    avg_x3 = Int(round(
                        sum(neighbor_x) / total
                    ))
                    avg_y3 = Int(round(
                        sum(neighbor_y) / total
                    ))

                    steering_force_x3 = Int(round(
                        (avg_x3 - boid[i].x - vx[i]) / cohesion_dial
                    ))
                    steering_force_y3 = Int(round(
                        (avg_y3 - boid[i].y - vy[i]) / cohesion_dial
                    ))

                    rx3[i],ry3[i]=clip_steering_force(steering_force_x3,steering_force_y3)

                end
            end
        end
    end
end

# update position of actors (boids)

function update(g::Game)
    global rx1, ry1, rx2, ry2, rx3, ry3
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
    rx1 = zeros(n)
    ry1 = zeros(n)
    rx2 = zeros(n)
    ry2 = zeros(n)
    rx3 = zeros(n)
    ry3 = zeros(n)

end

#tstopen
