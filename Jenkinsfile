pipeline {
    agent none

    triggers {
        pollSCM('H/5 * * * *')
    }

    options {
        skipDefaultCheckout()
        timestamps()
        timeout(time: 30, unit: 'MINUTES')
    }

    stages {
        stage('Build & Test') {
            // Mirrors the GitHub Actions fail-fast: false matrix
            failFast false
            parallel {

                stage('Ubuntu / GCC') {
                    agent { label 'linux' }
                    steps {
                        checkout scm
                        sh 'sudo apt-get update -qq && sudo apt-get install -y ninja-build g++'
                        sh '''
                            cmake -S . -B build -G Ninja \
                                -DCMAKE_BUILD_TYPE=Release \
                                -DCMAKE_CXX_COMPILER=g++ \
                                -DBALLISTICS_BUILD_TESTS=ON \
                                -DBALLISTICS_BUILD_EXAMPLES=OFF
                        '''
                        sh 'cmake --build build'
                        sh 'ctest --test-dir build --output-on-failure'
                    }
                    post {
                        always {
                            junit allowEmptyResults: true, testResults: 'build/**/*results*.xml'
                            cleanWs()
                        }
                    }
                }

                stage('Ubuntu / Clang') {
                    agent { label 'linux' }
                    steps {
                        checkout scm
                        sh 'sudo apt-get update -qq && sudo apt-get install -y ninja-build clang'
                        sh '''
                            cmake -S . -B build -G Ninja \
                                -DCMAKE_BUILD_TYPE=Release \
                                -DCMAKE_CXX_COMPILER=clang++ \
                                -DBALLISTICS_BUILD_TESTS=ON \
                                -DBALLISTICS_BUILD_EXAMPLES=OFF
                        '''
                        sh 'cmake --build build'
                        sh 'ctest --test-dir build --output-on-failure'
                    }
                    post {
                        always {
                            junit allowEmptyResults: true, testResults: 'build/**/*results*.xml'
                            cleanWs()
                        }
                    }
                }

                stage('Windows / MSVC') {
                    agent { label 'windows' }
                    steps {
                        checkout scm
                        bat '''
                            cmake -S . -B build ^
                                -DCMAKE_BUILD_TYPE=Release ^
                                -DBALLISTICS_BUILD_TESTS=ON ^
                                -DBALLISTICS_BUILD_EXAMPLES=OFF
                        '''
                        bat 'cmake --build build --config Release'
                        bat 'ctest --test-dir build --build-config Release --output-on-failure'
                    }
                    post {
                        always {
                            junit allowEmptyResults: true, testResults: 'build/**/*results*.xml'
                            cleanWs()
                        }
                    }
                }

            }
        }
    }

    post {
        failure {
            echo 'Pipeline failed — check stage logs above for details.'
        }
        success {
            echo 'All matrix builds passed.'
        }
    }
}
