// Returns true if this job is continuous or nightly.
def isProduction() {
  return ("${env.JOB_NAME}".contains("continuous") ||
    "${env.JOB_NAME}".contains("nightly"))
}

// Performs the checkout step for drake and drake-ci.
// * drake: Clones into WORKSPACE/'src' and checks out the branch
//   specified from the build.
// * drake-ci: Clones into WORKSPACE/'ci' and performs a custom
//   checkout of either 'main' or the given parameter.
def checkout(String ciSha = 'main') {
  def scmVars = null
  retry(4) {
      dir("${env.WORKSPACE}/src") {
        scmVars = checkout scm
      }
  }
  retry(4) {
    checkout([$class: 'GitSCM',
      branches: [[name: "${ciSha}"]],
      extensions: [[$class: 'AuthorInChangelog'],
        [$class: 'CloneOption', honorRefspec: true, noTags: true],
        [$class: 'RelativeTargetDirectory', relativeTargetDir: 'ci'],
        [$class: 'LocalBranch', localBranch: 'main']],
      userRemoteConfigs: [[
        credentialsId: 'ad794d10-9bc8-4a7a-a2f3-998af802cab0',
        name: 'origin',
        refspec: '+refs/heads/main:refs/remotes/origin/main',
        url: 'git@github.com:RobotLocomotion/drake-ci.git']]])
  }
  return scmVars
}

// Sends an email to Drake developers when a build fails or is unstable.
def emailFailureResults() {
  if (fileExists('RESULT')) {
    currentBuild.result = readFile 'RESULT'
    if (currentBuild.result == 'FAILURE' || currentBuild.result == 'UNSTABLE') {
      def subject = 'Build failed in Jenkins'
      if (currentBuild.result == 'UNSTABLE') {
        subject = 'Jenkins build is unstable'
      }
      emailext (
        subject: "${subject}: ${env.JOB_NAME} #${env.BUILD_NUMBER}",
        body: "See <${env.BUILD_URL}display/redirect?page=changes> " +
          "and <${env.BUILD_URL}changes>",
        to: '$DEFAULT_RECIPIENTS',
      )
    }
  }
}

// Deletes the workspace and tmp directories, for use at the end of a build.
def cleanWorkspace() {
  dir("${env.WORKSPACE}") {
    deleteDir()
  }
  dir("${env.WORKSPACE}@tmp") {
    deleteDir()
  }
}

// Provides links to CDash to view the results of the build.
def addCDashBadge() {
  if (fileExists('CDASH')) {
    def cDashUrl = readFile 'CDASH'
    addBadge icon: '/userContent/cdash.png',
      link: cDashUrl, text: 'View in CDash'
    addSummary icon: '/userContent/cdash.png',
      link: cDashUrl, text: 'View in CDash'
  }
}
