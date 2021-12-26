using GameZero , Colors

mutable struct Boid
    pos::Complex
    v::Complex
    a::Complex
    rxy:: Vector{Complex}
    color::RGB
end

perception_radius = 10    # 10
min_speed = 2             # 2
max_speed = 4             # 4
separation_dial = 60      # 60
alignment_dial = 8        # 8
cohesion_dial = 100       # 100

num_boids = 200
r=2
WIDTH = 1280
HEIGHT = 720
BACKGROUND = colorant"antiquewhite"
colors = [colorant"red", colorant"green", colorant"blue"]


xrange = [collect(-4:-2); collect(2:4)]

boid = [Boid(Complex(rand(20:5:(WIDTH - 20)),rand(20:5:(HEIGHT-20)))
              ,Complex(rand(xrange),rand(xrange))
              ,Complex(0,0)
              ,[Complex(0,0),Complex(0,0),Complex(0,0)]
              ,rand(colors))
        for i in 1:num_boids]

function draw(g::Game)
    for b in boid
        x = Int(round(real(b.pos)))
        y = Int(round(imag(b.pos)))
        draw(Circle(x,y,r),b.color, fill = true)
    end
end

function border(pos::Complex)
    xp,yp= real(pos),imag(pos)
    if xp > WIDTH
        xp = 0
    elseif xp < 0
        xp = WIDTH
    end
    if yp > HEIGHT
        yp=0
    elseif yp< 0
        yp = HEIGHT
    end
    return Complex(xp,yp)
end



function clip_steering_force(x)
    retval = x
     if(abs(x) < 1)
        retval=Int(round(min_speed * sign(x)))
    elseif abs(x) > max_speed
        retval = Int(round(max_speed * sign(x)))
    end
    return retval
end

function update_force(steering_force::Complex)
    return Complex(clip_steering_force(real(steering_force)),
                  clip_steering_force(imag(steering_force)))
end

function flock()
    separation_force = []
    neighbor_v = []
    neighbor = []

    total = 0
    for b in boid
        for b2 in boid
            deltaB = b.pos- b2.pos
            if b !== b2 && abs(deltaB) < perception_radius
                # populate arrays for separation rule
                push!(separation_force, deltaB)

                # populate arrays for alignment rule
                push!(neighbor_v, b2.v)
                # populate arrays for cohesion rule
                push!(neighbor, b2.pos)

                total += 1
                # conditional for boid within perception_radius
                #if total > 0
                    # 1. separation rule #######################################
                    avg1 = sum(separation_force)/total
                    b.rxy[1] = update_force((avg1-b.v)/separation_dial)

                    # 2. alignment rule ########################################
                    avg_v =sum(neighbor_v)/total
                    b.rxy[2] = update_force((avg_v - b.v)/alignment_dial)

                    # 3. cohesion rule #########################################
                    avg = sum(neighbor)/total
                    b.rxy[3] = update_force((avg - b.pos - b.v)/cohesion_dial)
                #end
            end
        end
    end
end
function update(g::Game)
    global rx1, ry1, rx2, ry2, rx3, ry3
    flock()

    for b in boid
        b.pos = border(b.pos)

        # calculate accelerations
        b.a  = sum(b.rxy)
        # update velocities
        b.v += b.a
       vxx = real(b.v)
       vyy=imag(b.v)
        if vxx == 0
            vxx = min_speed * rand((-1, 1))
        end
        if vyy  == 0
            vyy = min_speed * rand((-1, 1))
        end

        if abs(vxx) > max_speed
            vxx = max_speed * sign(vxx)
        end

        if abs(vyy) > max_speed
            vyy = max_speed * sign(vyy)
        end
        b.v = Complex(vxx,vyy)

        # update positions

        b.pos += b.v
        b.rxy=[Complex(0,0),Complex(0,0),Complex(0,0)]
    end
end
