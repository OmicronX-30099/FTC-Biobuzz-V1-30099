plugins {
    id("dev.frozenmilk.teamcode") version "12.0.0-1.2.2"
    id("dev.frozenmilk.sinister.sloth.load") version "0.3.2"
}

ftc {
    kotlin()
    sdk.TeamCode()
}

dependencies {
    implementation("com.pedropathing:revhub:3.0.1")
    implementation("com.pedropathing:tuning:1.0.1")
    implementation("com.pedropathing.ivy:pedro:1.1.1")
    implementation("dev.frozenmilk.sinister:Sloth:0.3.2")
// Slothboard preserves the com.acmerobotics.dashboard API and supports hot reload.
    implementation("com.acmerobotics.slothboard:dashboard:0.3.2+0.6.0")
}
