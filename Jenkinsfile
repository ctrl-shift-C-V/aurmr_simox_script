pipeline {
    agent any
    stages {
        stage('simox') {
            steps {
                script {
                    def BUILD_STATUS = sh(script: "rm -rf ~/.cmake && mkdir -p build && cd build && cmake .. && make", returnStatus: true)

                    if (BUILD_STATUS == 0) {
                        updateGitlabCommitStatus name: 'Build simox', state: 'success'    
                    } else {
                        updateGitlabCommitStatus name: 'Build simox', state: 'failed'
                        error("Build simox failed!")
                    }

                    def TEST_STATUS = sh(script: "cd build && ctest --output-on-failure", returnStatus: true)

                    if (TEST_STATUS == 0) {
                        updateGitlabCommitStatus name: 'Test simox', state: 'success'    
                    } else {
                        updateGitlabCommitStatus name: 'Test simox', state: 'failed'
                    }
                }
            }
        }

        stage('MMMCore') {
            steps {
                script {
                    def STATUS = build job: 'gitlab/MMMCore', wait: true, propagate: false

                    if (STATUS.buildVariables.BUILD_STATUS == "0") {
                        updateGitlabCommitStatus name: 'Build MMMCore', state: 'success'
                        if (STATUS.buildVariables.TEST_STATUS == "0") {
                            updateGitlabCommitStatus name: 'Test MMMCore', state: 'success'
                        } else {
                            updateGitlabCommitStatus name: 'Test MMMCore', state: 'failed'
                        }
                    } else {
                        updateGitlabCommitStatus name: 'Build MMMCore', state: 'failed'
                    }
                }
            }
        }

        stage('ArmarXCore') {
            steps {
                script {
                    def STATUS = build job: 'gitlab/ArmarXCore', wait: true, propagate: false

                    echo STATUS.buildVariables.BUILD_STATUS

                    if (STATUS.buildVariables.BUILD_STATUS == "0") {
                        updateGitlabCommitStatus name: 'Build ArmarXCore', state: 'success'
                        if (STATUS.buildVariables.TEST_STATUS == "0") {
                            updateGitlabCommitStatus name: 'Test ArmarXCore', state: 'success'
                        } else {
                            updateGitlabCommitStatus name: 'Test ArmarXCore', state: 'failed'
                        }
                    } else {
                        updateGitlabCommitStatus name: 'Build ArmarXCore', state: 'failed'
                    }
                }
            }
        }

        stage('MMMTools') {
            steps {
                script {
                    def STATUS = build job: 'gitlab/MMMTools', wait: true, propagate: false

                    if (STATUS.buildVariables.BUILD_STATUS == "0") {
                        updateGitlabCommitStatus name: 'Build MMMTools', state: 'success'
                        if (STATUS.buildVariables.TEST_STATUS == "0") {
                            updateGitlabCommitStatus name: 'Test MMMTools', state: 'success'
                        } else {
                            updateGitlabCommitStatus name: 'Test MMMTools', state: 'failed'
                        }
                    } else {
                        updateGitlabCommitStatus name: 'Build MMMTools', state: 'failed'
                    }
                }
            }
        }

    }
}
