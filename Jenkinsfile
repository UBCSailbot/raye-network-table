// working directory
pipeline {
    agent none
    stages {
        stage('Build BBB and network-table') {
            agent {
                label 'bbb_2'
            }
            steps {
                dir("/home/debian/network-table") {
	                git credentialsId: '371ce800-27d7-424a-a2c9-98aa355f0031', url: 'https://github.com/UBCSailbot/network-table'
                }
                echo "SUCCESS"
                echo env.BRANCH_NAME
                echo env.CHANGE_URL
                echo env.CHANGE_ID
            }
        }
    }
}
