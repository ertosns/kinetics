/*
//adhock
TEST(MANIPULACTION, analyseContacts) {
    int N, x, y, n;
    cin >> N;
    int Forces[N][3];
    Eigen::Vector3d F[N];
    for (int i=0; i < N; i++) {
        cin >> x >> y >> n;
        float angle=(n*M_PI)/180; //from degrees to radians
        //contacts[i][0] = -1*1*cos(angle)-1*x*sin(angle);
        //contacts[i][1] = cos(angle);
        //contacts[i][2] = sin(angle);
        //
        F[i][0] = round(y*cos(angle)+x*sin(angle));
        F[i][1] = round(cos(angle));
        F[i][2] = round(sin(angle));
        //
    }
    Eigen::MatrixXd W(3,N);
    for (int i=0; i < N; i++) {
        W.col(i) = F[i];
    }
    std::cout << "Wrench: " << W << std::endl;
    //
    Eigen::VectorXd K;
    auto f = Eigen::VectorXd::Ones(N);
    auto Aeq = W;
    auto beq = Eigen::Vector3d::Zero();
    auto Aieq = -1*Eigen::Matrix4d::Identity(N, N);
    auto bieq = -1*Eigen::VectorXd::Ones(N);
    bool res;
    try {
        res = form_closure(f, Aieq, bieq, Aeq, beq, K);
    } catch(NotFormClosure e) {
        std::cerr << e.what() << std::endl;
    } catch(exception e) {
        std::cerr << e.what() << std::endl;
    }
    std::cout << "the system in contact in a closed form: " << ((res) ? "TRUE":"FALSE") << std::endl;
}

//THIS IS JUST AN ADHOCK
TEST(MANIPULATION, assembly) {
    int N, M, rb1, rb2;
    float x, y, m, mau, normal;
    float g=0.981;
    cin >> N >> M;
    std::vector<Eigen::Vector3d> contacts[N+1][N+1];
    auto ZERO_VEC = Eigen::Vector3d::Zero();
    //read contact points, and masses
    //for (int i=0; i <= N; i++) fill(contacts[i], contacts[i]+N+1, ZERO_VEC);
    int mass[N+1][3];
    for (int i=1; i <=N; i++){
        cin >> x >> y >> m;
        mass[i][0]=x;
        mass[i][1]=y;
        mass[i][2]=m;
    }
    //populated contacts, and contact reactions
    for (int i=0;i < M; i++) {
        cin >> rb1 >> rb2 >> x >> y >> normal >> mau;
        //
        float angle=(normal*M_PI)/180;
        std::cout << "contact's normal: " << angle << std::endl;
        auto p = Eigen::Vector3d(x, y, 0);
        std::cout << "contact's position p: " << p << std::endl;
        Eigen::Vector3d f_uvec = mau*Eigen::Vector3d(cos(angle), sin(angle), 0);
        std::cout << "contact's force f: " << f_uvec << std::endl;
        std::cout << f_uvec(0) << endl << f_uvec(1) << endl;
        float m = (Algebra::VecToso3(p)*f_uvec)(2);
        //float m =0;
        std::cout << "contact wrench's moment m: " << m << std::endl;
        std::cout << "contact's force f: " << f_uvec << std::endl;
        auto F = Eigen::Vector3d(m, f_uvec(0), f_uvec(1));
        std::cout << " contact wrench F: " << F << std::endl;
        //action
        contacts[rb2][rb1].push_back(F);
        //reaction
        contacts[rb1][rb2].push_back(-1*F);
    }
    //process contacts
    std::cout << "processing contacts" << std::endl;
    for (int i=1; i <= N; i++) {
        //calculate the form closure for reach rigid body.
        int c=0;
        for (int j=0; j <= N; j++) {
            //how many contact point acting on this rigid body?
            //if (contacts[i][j]!=ZERO_VEC) c++;
            c+=contacts[i][j].size();
        }
        ASSERT_TRUE(c>0); //at least one non-zero force required for calculation
        std::cout << "number of contacts acting on rigid body (" << i
                  << ") is " << c << std::endl;
        //wrench acting on current rigid body
        Eigen::MatrixXd W = Eigen::MatrixXd(3,c);
        int wrench_idx=0;
        for (int j=0; j<=N; j++) {
            //if (contacts[i][j]!=ZERO_VEC)
            for (int k=0; k<contacts[i][j].size(); k++)
                W.col(wrench_idx++)=contacts[i][j][k];
        }
        std::cout << "wrench: " << W << endl;
        auto k = Eigen::VectorXd(c);
        auto f = Eigen::VectorXd::Ones(c);
        auto Aeq = W;
        //TODO  (fix) that isn't he wrench!! but the force
        //you need the position of the center of mass to know the external force due to gravity
        auto com = Eigen::Vector3d(mass[i][0], mass[i][1], 0);
        auto g_force = Eigen::Vector3d(0,0,-1*mass[i][2]*g);
        auto beq = -1*Algebra::VecToso3(com)*g_force;
        auto Aieq = W;
        auto bieq = Eigen::VectorXd::Zero(c);
        std::cout << "K: " << k << endl
                  << "f: " << f << endl
                  << "Aeq: " << Aeq << endl
                  << "beq: " << beq << endl
                  << "Aieq: " << Aieq << endl
                  << "bieq: " << bieq << endl;
        bool isclosure=false;
        try {
            isclosure=form_closure(f, Aieq, bieq, Aeq, beq, k);
        } catch(exception e) {
            std::cerr << e.what() << std::endl;
        }
        ASSERT_TRUE(isclosure);
    }
}
*/
