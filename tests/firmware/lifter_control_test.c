#include <assert.h>
#include <stdio.h>
#include "lifter_control.h"

static const lifter_config_t config = {
    .enabled=true, .lower_deg=0, .upper_deg=80, .pulse_0_us=500, .pulse_180_us=2500,
    .up_ms=600, .down_ms=600, .timeout_ms=5000,
};
static lifter_control_t fresh(void)
{
    lifter_control_t l;
    lifter_control_init(&l, &config);
    assert(l.state==LIFTER_IDLE && l.position==-1 && l.target_deg==-1 && l.pulse_us==0);
    return l;
}
int main(void)
{
    assert(lifter_control_angle_pulse(&config,0)==500);
    assert(lifter_control_angle_pulse(&config,80)==1389);
    assert(lifter_control_angle_pulse(&config,90)==1500);
    assert(lifter_control_angle_pulse(&config,180)==2500);
    lifter_control_t l=fresh();
    assert(lifter_control_command(&l,2,true,0)==LIFTER_BAD_VALUE);
    assert(lifter_control_command(&l,1,false,0)==LIFTER_WHEELS_MOVING);
    // The first command works without arm and without an assumed startup angle.
    assert(lifter_control_command(&l,1,true,10000)==LIFTER_OK);
    assert(lifter_control_blocks_wheels(&l) && l.pulse_us==1389 && l.target_deg==80);
    for(int64_t t=20000;t<610000;t+=10000) {
        assert(lifter_control_command(&l,1,false,t)==LIFTER_DUPLICATE);
        lifter_control_tick(&l,t,true,false);
    }
    assert(l.end_us==610000 && l.completed==0);
    assert(lifter_control_command(&l,0,true,610000)==LIFTER_BUSY);
    lifter_control_tick(&l,610000,true,false);
    assert(l.state==LIFTER_IDLE && l.position==1 && l.completed==1 && l.pulse_us==1389);
    // Regression: completion must keep 80 degrees, never write the old 1500us.
    for(int64_t t=620000;t<5000000;t+=10000) lifter_control_tick(&l,t,true,false);
    assert(l.pulse_us==1389 && l.state==LIFTER_IDLE && l.target_deg==80);
    assert(lifter_control_command(&l,1,true,5000000)==LIFTER_DUPLICATE);
    assert(lifter_control_command(&l,0,true,5010000)==LIFTER_OK && l.pulse_us==500);
    lifter_control_tick(&l,5610000,true,false);
    assert(l.position==0 && l.completed==2 && l.pulse_us==500);
    lifter_control_tick(&l,9000000,true,false);
    assert(l.pulse_us==500); // keep lower angle too
    l=fresh();
    assert(lifter_control_command(&l,0,true,0)==LIFTER_OK); // initial lower command must execute

    lifter_config_t c=config;
    c.lower_deg=20; c.upper_deg=100; c.pulse_0_us=2500; c.pulse_180_us=500;
    lifter_control_init(&l,&c);
    assert(lifter_control_command(&l,1,true,0)==LIFTER_OK && l.target_deg==100 && l.pulse_us==1389);
    c=config; c.enabled=false; lifter_control_init(&l,&c);
    assert(l.state==LIFTER_DISABLED && !lifter_control_blocks_wheels(&l));
    assert(lifter_control_command(&l,1,true,0)==LIFTER_NOT_READY && !l.pulse_us);
    c=config; c.lower_deg=c.upper_deg; lifter_control_init(&l,&c);assert(l.state==LIFTER_UNCONFIGURED);
    c=config; c.upper_deg=181; lifter_control_init(&l,&c);assert(l.state==LIFTER_UNCONFIGURED);
    c=config; c.pulse_180_us=500; lifter_control_init(&l,&c);assert(l.state==LIFTER_UNCONFIGURED);
    c=config; c.up_ms=c.timeout_ms; lifter_control_init(&l,&c);assert(l.state==LIFTER_UNCONFIGURED);
    c=config; c.down_ms=0; lifter_control_init(&l,&c);assert(l.state==LIFTER_UNCONFIGURED);

    l=fresh(); lifter_control_command(&l,1,true,0);
    lifter_control_stop(&l,LIFTER_STOPPED);
    assert(l.state==LIFTER_FAULT && l.pulse_us==0 && l.position==-1 && l.target_deg==-1);
    assert(lifter_control_blocks_wheels(&l));
    assert(lifter_control_command(&l,0,true,10000)==LIFTER_OK && l.pulse_us==500);
    l=fresh(); lifter_control_command(&l,1,true,0);
    lifter_control_tick(&l,631000,true,false);
    assert(l.fault==LIFTER_TIMEOUT && l.completed==0 && l.pulse_us==0);
    lifter_control_stop(&l,LIFTER_STOPPED);
    assert(l.fault==LIFTER_TIMEOUT && lifter_control_command(&l,0,true,700000)==LIFTER_NOT_READY);
    l=fresh(); lifter_control_command(&l,1,true,0);
    lifter_control_tick(&l,600000,true,false);
    lifter_control_tick(&l,700000,false,false); // lease failure while holding also releases PWM
    assert(l.fault==LIFTER_COMM_LOST && l.pulse_us==0);
    l=fresh(); lifter_control_command(&l,1,true,0);
    lifter_control_tick(&l,100000,true,true);
    assert(l.fault==LIFTER_WHEEL_FAULT && l.pulse_us==0);
    puts("PASS: direct positional targets, 0/80 mapping, persistent hold, no arm, duplicate/deadline/fault behavior");
}
