    using Statistics ,Luxor
    randangle(m) = 2* pi*rand(Float64,m)
    complex2Point(scale,z) = Point.(scale .* real(z),scale .* imag(z))
    function runit(k)
        theta = randangle(k)
        r = sqrt.(rand(Float64,k))
        p =  r .* complex.(cos.(theta),sin.(theta))
        x = sqrt.( abs.(p).^(-2) .- 1)
        z1 ,z2=   p + (im)x .* p , p - (im)x .* p
        p1,p2 = complex2Point(300,z1), complex2Point(300,z2)

        @png begin
            Drawing(700,700)
            origin()
            setcolor("white")
            line.(p1,p2,:stroke)
            circle.(300 .* real(p),300 .* imag(p),5,:stroke)
        end
    end

    function runit2(n)
        theta1,theta2 = randangle(n), randangle(n)
        z1,z2 = complex.(cos.(theta1),sin.(theta1)), complex.(cos.(theta2),sin.(theta2))
        p1,p2 = complex2Point(300,z1), complex2Point(300,z2)
        print(length(filter(x -> x > sqrt(3), abs.(z2-z1)))/n)

        @png begin
            Drawing(700,700)
            origin()
            setcolor("white")

            mp = midpoint.(p1,p2)
            #line.(p1,p2,:stroke)
            #circle.(p1,3,:stroke)
            #circle.(p2,3,:stroke)
            circle.(mp,3,:stroke)
        end
    end


runit2(2090)
