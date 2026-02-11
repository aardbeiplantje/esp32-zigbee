#!/usr/bin/perl
#
# ble_uart.pl - BLE UART (Nordic UART Service) bridge in Perl
#
# This script connects to one or more BLE devices implementing the Nordic UART
# Service (NUS), providing a terminal interface for interactive communication
# over Bluetooth Low Energy. Supports multiple simultaneous connections,
# scripting, and AT command mode for ESP32/ESP-AT devices.
#
# This is free and unencumbered software released into the public domain.
#
# Anyone is free to copy, modify, publish, use, compile, sell, or
# distribute this software, either in source code form or as a compiled
# binary, for any purpose, commercial or non-commercial, and by any
# means.
#
# In jurisdictions that recognize copyright laws, the author or authors
# of this software dedicate any and all copyright interest in the
# software to the public domain. We make this dedication for the benefit
# of the public at large and to the detriment of our heirs and
# successors. We intend this dedication to be an overt act of
# relinquishment in perpetuity of all present and future rights to this
# software under copyright law.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# For more information, please refer to <https://unlicense.org>
#

BEGIN {
    # sadly this doesn't work in a BEGIN block, just keep it for reference
    #$ENV{LC_ALL} //= "C";
};

# set up $0, our application name, so the process shows up nicely in "ps" output
BEGIN {
    $::APP_NAME = "ble:uart";
    $::DOLLAR_ZERO = $0;
    $0 = $::APP_NAME;
};

# Experimental attempt at reducing memory usage, perl classes are already
# pretty fine, but no need to use e.g. "Storable" or "Carp" if we don't need
# them.
#
# Also, "common::sense" is just a bad joke of a module, don't use it.
#
# Similarly, "Carp" is just a waste of memory for what we need here, regular
# "die" is a perl builtin and much lighter.
BEGIN {
    # clean up INC to avoid stupid memory usage (although "Encode" just sucks at it)
      $INC{"Storable.pm"}
    = $INC{"Types/Serialiser.pm"}
    = $INC{"common/sense.pm"}
    = $INC{"Carp.pm"}
    = 1;
    # low memory usage "Carp"
    package Carp;
    sub import {
        my $c = (caller(0))[0];
        no strict 'refs';
        *{"${c}::croak"}     =
        *{"${c}::confess"}   =
        *{"${c}::carp"}      =
        *{"${c}::cluck"}     =
        *{"${c}::longmess"}  =
        *{"${c}::shortmess"} = \&CORE::die;
        return;
    }
    *croak     =
    *confess   =
    *carp      =
    *cluck     =
    *longmess  =
    *shortmess = \&CORE::die;
};

use strict; use warnings;
no warnings 'once';


# app/loop state and config, a global instance variable
$::APP_OPTS = handle_cmdline_options();

# start the application
eval {
    main_loop();
};
if($@){
    chomp(my $err = $@);
    if($err =~ m/^(TERM|INT) signal, exiting$/){
        logger::info("exiting cleanly: $err");
        exit 0;
    } else {
        logger::error("main loop error: $err");
        exit 1;
    }
}
logger::info("exiting");

END {
    # make sure the terminal is clean and reset again
    print $colors::reset_color if utils::cfg('interactive_color', 1);
}

exit;

sub main_loop {

    # for possible  socket problems, although we do non blocking very fast
    local $SIG{PIPE} = "IGNORE";
    # makes syscalls restarted
    local $SIG{HUP}  = "IGNORE";

    # init global constants
    $::APP_CONN = {};
    $::CURRENT_CONNECTION = undef;
    $::COMMAND_BUFFER     = undef;

    # our clean exiting
    $::DATA_LOOP = 1;
    local $SIG{INT}  =
    local $SIG{TERM} = sub {
        $::DATA_LOOP = 0;
        die "$_[0] signal, exiting\n"
    };

    # we start non-sleepy
    my $s_timeout;

    # select vecs
    my ($rin, $win, $ein);

    # initialize our targets
    my $tgts = $::APP_OPTS->{targets} // [];
    logger::debug("starting main loop with targets", $tgts);
    connect_tgt($::APP_CONN, $_) for @{$tgts};

    # our input reader - choose between TTY and STDIN based on whether STDIN is a terminal
    my $reader = (-t STDIN and !utils::cfg('raw')) ? input::tty->new() : input::stdin->new();
    my $response_buffer = "";
    my $color_ok = $::APP_OPTS->{_color_ok};
    my $c_reset = $colors::reset_color;
    $c_reset = "" unless $color_ok;
    my ($e_color, $s_color) = ("", "");
    $e_color = $colors::bright_red_color    if $color_ok;
    $s_color = $colors::bright_yellow_color if $color_ok;

    # a message printer subroutine, different for STDIN vs TTY, and use
    # closures for less cpu cycle use (reuse precomputed stuff and store it)
    my $msg_printer;
    if(utils::cfg('raw')){
        $msg_printer = sub {
            my ($data_ref) = @_;
            my $r = syswrite($reader->outfh(), $$data_ref);
            if(!defined $r){
                logger::error("syswrite error: $!");
                $::DATA_LOOP = 0; # exit the main loop
                return;
            }
            return;
        };
    } else {
        my $ttydisplaybuffer = "";
        my $c_info = "";
        $msg_printer = sub {
            my ($data_ref) = @_;
            return unless length($$data_ref//"");
            logger::debug(">>TTY>> showing message, length:".length($$data_ref));

            # append to display buffer
            $ttydisplaybuffer .= $$data_ref;

            # remote info
            my $b_addr = "";
            $b_addr = $::APP_OPTS->{_utf8_ok} ? "❰$::CURRENT_CONNECTION->{cfg}{b}❱❱ " : "[$::CURRENT_CONNECTION->{cfg}{b}] "
                if defined $::CURRENT_CONNECTION;
            $b_addr = $colors::dark_yellow_color.$b_addr if $color_ok and length($b_addr);

            # command info
            if(defined $::COMMAND_BUFFER and utils::cfg("interactive_command_info", 1)){
                $c_info = $::COMMAND_BUFFER;
                chomp($c_info);
                $c_info = $::APP_OPTS->{_utf8_ok} ? "⦗$c_info⦘ ": "[$c_info] ";
                $c_info = $colors::bright_blue_color3.$c_info.$c_reset if $color_ok;
                $::COMMAND_BUFFER = undef;
            }

            logger::debug(">>TTY>> display buffer length: ", length($ttydisplaybuffer), " bytes >>$ttydisplaybuffer<<");
            while($ttydisplaybuffer =~ s/(.*?\r?\n)//){
                my $l = $1;
                last unless length($l//"");
                logger::debug(">>TTY>> showing one line: $l");
                $l =~ s/\r?\n$//;
                my $c_resp  = $l =~ m/^\+ERROR:/ ? $e_color : $s_color;
                $c_resp = "" unless $color_ok;
                $reader->show_message($::APP_OPTS->{_reply_line_prefix}.$b_addr.$c_info.$c_resp.$l.$c_reset);
            }
            return;
        };
    }

    # shuffler sub? note that "keys" in perl randomizes already since perl 5.18
    my $shuffler_sub = sub {@_};
    if(utils::cfg("shuffle_connections")){
        utils::load_cpan("List::Util");
        $shuffler_sub = \&List::Util::shuffle;
    }


    $::OUTBOX = [];
    # If --script was given, run it before entering the main loop
    handle_command("/script $::APP_OPTS->{script}") if $::APP_OPTS->{script};

    # main loop
    eval {
    while($::DATA_LOOP){

        # check all connections, and if they have an empty outbox, exit
        logger::debug("main loop iteration, ", scalar(keys %{$::APP_CONN}), " connections, ",
                      (defined $::COMMAND_BUFFER ? "waiting for command response, " : ""),
                      scalar(@{$::OUTBOX}), " messages in outbox, exit wanted? ".($::DATA_LOOP_EXIT_WANTED?"yes":"no"));
        if($::DATA_LOOP_EXIT_WANTED){
            my $all_empty = 1;
            foreach my $c (values %{$::APP_CONN}){
                logger::debug("checking connection $c->{_log_info} (fd: $c->{_fd}), buffer: ", length($c->{_outboxbuffer}//""), " bytes");
                if(length($c->{_outboxbuffer}//"") > 0){
                    $all_empty = 0;
                    last;
                }
            }
            if($all_empty and !defined $::COMMAND_BUFFER and !@{$::OUTBOX}){
                logger::info("all connections have empty outbox, exiting");
                $::DATA_LOOP = 0;
                last;
            }
        }

        # reset select() vecs each loop iteration
        $rin = "";
        $win = "";
        $ein = "";

        # input, note: STDIN is FD=0, so use defined check here
        vec($rin, $reader->infd(), 1) = 1 if defined $reader->infd() and !defined $::COMMAND_BUFFER;

        # select() vec handling
        foreach my $fd (&{$shuffler_sub}(keys %{$::APP_CONN})){
            logger::debug("checking connection fd=$fd");
            my $c = $::APP_CONN->{$fd};
            vec($rin, $fd, 1) = 1;

            # other stuff to write?
            vec($win, $fd, 1) = $c->need_write();

            # need select() timeout
            my $tm = $c->need_timeout();
            $s_timeout = $tm if defined $tm and (!defined $s_timeout or (defined $s_timeout and $tm < $s_timeout));
        }

        # next select timeout? if we are missing connections, set to 1s, else undef (wait forever)
        my $n_conns = (keys %{$::APP_CONN} != @{$tgts}) ? 1 : undef;
        $s_timeout = $n_conns if !defined $s_timeout or (defined $s_timeout and defined $n_conns and $n_conns < $s_timeout);
        $s_timeout = 0 if defined $s_timeout and $s_timeout < 0;

        # reader timeout?
        $s_timeout = 0 if defined $::CURRENT_CONNECTION and @{$::OUTBOX} and !defined $::COMMAND_BUFFER and !defined $s_timeout;

        # we're waiting for a command response? timeout short to run the spinner
        $s_timeout = 0.1 if defined $::COMMAND_BUFFER and defined $s_timeout and $s_timeout > 0.1;
        logger::debug(">>TTY>> setting select timeout to", $s_timeout);

        # do select() call, waiting for changes
        $ein |= $rin | $win;
        my $r = select(my $rout = $rin, my $wout = $win, my $eout = $ein, $s_timeout);
        if($r == -1){
            $! == e::EINTR() or $! == e::EAGAIN() or logger::error("select problem: $!");
            next;
        }

        # check IN|OUT for reader
        if(vec($rout, $reader->infd(), 1)){
            $reader->do_read();
        }

        # process reader's OUTBOX messages if there are any
        if(defined $::CURRENT_CONNECTION){
            if(!defined $::COMMAND_BUFFER and defined(my $cmd_data = shift @{$::OUTBOX})){
                logger::debug(">>TTY>>", length($cmd_data), " bytes read from TTY");
                my $r_ok = handle_command($cmd_data);
                if(!defined $r_ok){
                    logger::info("SEND: ", $cmd_data);
                    $::CURRENT_CONNECTION->{_outboxbuffer} .= $cmd_data;
                    $::COMMAND_BUFFER = $cmd_data;
                }
            }
        } else {
            if(@{$::OUTBOX}){
                logger::debug(">>TTY>> have ", scalar(@{$::OUTBOX}), " messages in outbox, but no current connection");
                if(-t STDIN and !utils::cfg('raw')){
                    $reader->show_message("${e_color}No current connection set, cannot send data$c_reset");
                }
            }
        }

        # anything to read from the remote connections?
        foreach my $fd (&{$shuffler_sub}(keys %{$::APP_CONN})){
            my $c = $::APP_CONN->{$fd};
            eval {
                if(vec($eout, $fd, 1)){
                    # check error
                    my $err = getsockopt($c->{_socket}, ble::SOL_SOCKET(), ble::SO_ERROR());
                    if(!defined $err){
                        logger::error("Error getting socket options: $!");
                    } else {
                        $! = unpack("I", $err);
                        if($!){
                            logger::error("Socket error on $c->{_log_info} (fd: $fd): $!");
                            removing_tgt($::APP_CONN, $c);
                            return;
                        }
                    }
                    removing_tgt($::APP_CONN, $c);
                    return;
                }

                # handle read if select says so
                if(vec($rout, $fd, 1)){
                    my $read_ok = $c->do_read(\$response_buffer);
                    if(!$read_ok){
                        # EOF
                        removing_tgt($::APP_CONN, $c);
                        return;
                    }
                }

                # handle write if select says so
                $c->do_write() if vec($wout, $fd, 1);
            };
            if($@){
                chomp(my $err = $@);
                do {$::DATA_LOOP = 0; last} if $err =~ m/^QUIT at .* line \d+/;
                logger::error($err);
                removing_tgt($::APP_CONN, $c);
            }
        }

        # check connections
        if(keys %{$::APP_CONN} != @{$tgts}){
            # we have missing connections, try to reconnect
            foreach my $t (@{$tgts}){
                next if grep {$t->{b} eq $_->{b}} values %{$::APP_CONN};

                # was it the current connection?
                if(defined $::CURRENT_CONNECTION and $::CURRENT_CONNECTION->{cfg}{b} eq $t->{b}){
                    logger::info("current connection $t->{b} disconnected, clearing current connection");
                    $::CURRENT_CONNECTION = undef;
                }

                # try to reconnect
                eval {connect_tgt($::APP_CONN, $t)};
                if($@){
                    chomp(my $err = $@);
                    do {$::DATA_LOOP = 0; last} if $err =~ m/^QUIT at .* line \d+/;
                    logger::error("problem reconnecting [$t->{b}]: $err");
                }
            }
        } elsif(!defined $::CURRENT_CONNECTION){
            if(grep {$_->is_connected()} values %{$::APP_CONN}){
                logger::info("setting current connection to first target: $tgts->[0]{b}");
                $::CURRENT_CONNECTION = (grep {$_->{cfg}{b} eq $tgts->[0]{b}} values %{$::APP_CONN})[0];
            } else {
                logger::debug("no targets connected, cannot set current connection");
            }
        }

        # if we have a response buffer, write it to the TTY
        if(length($response_buffer) > 0){
            my $resp = substr($response_buffer, 0, length($response_buffer), '');
            logger::debug(">>TTY>>", length($resp), " bytes to write to TTY");
            &{$msg_printer}(\$resp);
        }

        # if we have a COMMAND_BUFFER, but no response, we spin the prompt
        if(utils::cfg("interactive_command_info", 0)){
            if(defined $::COMMAND_BUFFER and !length($response_buffer)){
                $reader->spin();
            }
        } else {
            if($::CURRENT_CONNECTION and !length($::CURRENT_CONNECTION->{_outboxbuffer}//"")){
                $::COMMAND_BUFFER = undef;
            }
        }

        $s_timeout = undef; # reset timeout for next iteration
    }
    };
    chomp(my $err = $@);

    # handle clean exits
    removing_tgt($::APP_CONN, $_) for values %{$::APP_CONN};

    # cleanup reader
    $reader->cleanup();

    # relay error if any
    die "$err\n" if $err and $err !~ m/^(TERM|INT) signal, exiting$/;
    return
}

sub removing_tgt {
    my ($conns, $c) = @_;
    return unless defined $c and defined $c->{_fd};
    logger::info("cleanup $c->{_log_info} (fd: $c->{_fd})");
    delete $conns->{$c->{_fd}};
    $c->cleanup();
    return;
}

sub connect_tgt {
    my ($conns, $c, $blocking) = @_;
    my $n = ble::uart->new({%$c});
    my $fd = $n->init($blocking);
    return unless defined $fd;
    $n->blocking(0);
    $conns->{$fd} = $n;
    return;
}

# Helper function to handle command line options, general options, not per target configuration
sub handle_cmdline_options {
    my $cfg = {};

    # raw mode? skip Getopt::Long if so
    $cfg->{raw} = (utils::cfg('raw') or grep {/^--?raw|r$/} @ARGV) ? 1 : 0;
    @ARGV = grep {!/^--?raw|r$/} @ARGV;
    utils::set_cfg('raw', $cfg->{raw});

    # do we have options that warrant the Getopt::Long parsing?
    if(grep {/^--?(manpage|man|m|help|h|\?|script)$/} @ARGV){
        utils::load_cpan("Getopt::Long");
        Getopt::Long::Configure("bundling", "no_ignore_case", "pass_through");
        Getopt::Long::GetOptions(
            $cfg,
            "raw|r!",
            "manpage|man|m!",
            "help|h|?!",
            "script=s",
        ) or utils::usage(-exitval => 1);
        utils::usage(-verbose => 1, -exitval => 0) if $cfg->{help};
        utils::manpage(1) if $cfg->{manpage};
    }

    # default options for the TTY reply line
    $cfg->{_reply_line_prefix} = "";

    # raw or not?
    if(utils::cfg("raw")){
        utils::set_cfg('loglevel', 'NONE') unless defined utils::cfg('loglevel');
    } else {
        # color support?
        if(utils::cfg("interactive_color", 1)){
            $cfg->{_color_ok} = 1 if ($ENV{COLORTERM}//"") =~ /color/i or ($ENV{TERM}//"") =~ /color/i;
        }

        # UTF-8 support?
        $cfg->{_utf8_ok} = 1 if utils::cfg('interactive_utf8', 1);
        $cfg->{_reply_line_prefix} = $cfg->{_utf8_ok} ? "↳ " : "> ";
        if($cfg->{_color_ok}){
            $cfg->{_reply_line_prefix} = $colors::green_color.$cfg->{_reply_line_prefix}.$colors::reset_color;
        }
    }

    # parse the cmdline options for targets to connect to
    $cfg->{targets} = [];
    while(defined($_ = shift @ARGV)){
        next if $_ eq '';
        last if $_ eq '--'; # stop processing targets on --
        add_target($cfg, $_, 0) or utils::usage(-exitval => 1);
    }

    return $cfg;
}

# Helper function to validate static random address
sub is_valid_static_random_address {
    my ($addr) = @_;
    # Extract the first octet
    my ($first_octet) = $addr =~ /^([0-9A-Fa-f]{2})/;
    return 0 unless defined $first_octet;

    my $first_byte = hex($first_octet);
    # Static random addresses must have the two most significant bits set to '11'
    # This means the first octet must be in range 0xC0-0xFF
    return (($first_byte & 0xC0) == 0xC0);
}

# Helper function to detect address type based on MAC address
sub detect_address_type {
    my ($addr) = @_;
    # Extract the first octet
    my ($first_octet) = $addr =~ /^([0-9A-Fa-f]{2})/;
    return 1 unless defined $first_octet; # BDADDR_LE_PUBLIC

    my $first_byte = hex($first_octet);

    # Check if it's a static random address (two MSBs = 11)
    if (($first_byte & 0xC0) == 0xC0) {
        return 2; # BDADDR_LE_RANDOM
    }

    # Check for other random address indicators
    # Private resolvable addresses have MSBs = 01 (0x40-0x7F)
    # Private non-resolvable addresses have MSBs = 00 (0x00-0x3F)
    if (($first_byte & 0xC0) == 0x40) {
        logger::warn("Address $addr appears to be a private resolvable address - treating as random");
        return 2; # BDADDR_LE_RANDOM
    }

    if (($first_byte & 0xC0) == 0x00) {
        logger::warn("Address $addr appears to be a private non-resolvable address - treating as random");
        return 2; # BDADDR_LE_RANDOM
    }

    # Default to public for everything else
    return 1; # BDADDR_LE_PUBLIC
}

# Helper function to get detailed address type information
sub get_random_address_subtype {
    my ($addr) = @_;
    my ($first_octet) = $addr =~ /^([0-9A-Fa-f]{2})/;
    return "unknown" unless defined $first_octet;

    my $first_byte = hex($first_octet);

    if (($first_byte & 0xC0) == 0xC0) {
        return "static random";
    } elsif (($first_byte & 0xC0) == 0x40) {
        return "private resolvable";
    } elsif (($first_byte & 0xC0) == 0x00) {
        return "private non-resolvable";
    } else {
        return "invalid random";
    }
}

# Helper function to get address type name for logging
sub get_address_type_name {
    my ($addr_type) = @_;
    return "public" if $addr_type == 1; # BDADDR_LE_PUBLIC
    return "random" if $addr_type == 2; # BDADDR_LE_RANDOM
    return "unknown($addr_type)";
}

# Helper to map a security profile to the constant
sub get_security_profile {
    my ($profile) = @_;
    $profile //= 'low';
    $profile = lc($profile);
    return 0 if $profile eq 'none';   # BT_SECURITY_SDP
    return 1 if $profile eq 'low';    # BT_SECURITY_LOW
    return 2 if $profile eq 'medium'; # BT_SECURITY_MEDIUM
    return 3 if $profile eq 'high';   # BT_SECURITY_HIGH
    return 4 if $profile eq 'fips';   # BT_SECURITY_FIPS
    return 1; # default to BT_SECURITY_LOW
}

# Helper to map the io capability constant to a name
sub io_capability_name {
    my ($cap) = @_;
    $cap //= 3; # default to BT_IO_CAP_NO_INPUT_OUTPUT
    $cap = lc($cap);
    return 0 if $cap == 'display-only';     # BT_IO_CAP_DISPLAY_ONLY
    return 1 if $cap == 'display-yesno';    # BT_IO_CAP_DISPLAY_YESNO
    return 2 if $cap == 'keyboard-only';    # BT_IO_CAP_KEYBOARD_ONLY
    return 3 if $cap == 'no-input-output';  # BT_IO_CAP_NO_INPUT_OUTPUT
    return 4 if $cap == 'keyboard-display'; # BT_IO_CAP_KEYBOARD_DISPLAY
    return 3; # default to BT_IO_CAP_NO_INPUT_OUTPUT
}

# Helper function to validate any BLE address format
sub is_valid_ble_address {
    my ($addr, $expected_type) = @_;

    # Basic format check
    return 0 unless $addr =~ /^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$/;

    # If no type specified, any valid format is ok
    return 1 unless defined $expected_type;

    if ($expected_type == 1) { # BDADDR_LE_PUBLIC
        # Public addresses can be any format (no special restrictions)
        return 1;
    } elsif ($expected_type == 2) { # BDADDR_LE_RANDOM
        # For random type, validate it's a proper random address
        my ($first_octet) = $addr =~ /^([0-9A-Fa-f]{2})/;
        return 0 unless defined $first_octet;
        my $first_byte = hex($first_octet);

        # Check if it matches any valid random address pattern
        return 1 if ($first_byte & 0xC0) == 0xC0; # Static random
        return 1 if ($first_byte & 0xC0) == 0x40; # Private resolvable
        return 1 if ($first_byte & 0xC0) == 0x00; # Private non-resolvable

        return 0;  # Invalid random address
    }

    return 0;  # Unknown type
}

sub add_target {
    my ($cfg, $tgt, $blocking) = @_;
    my ($addr, $opts) = ($tgt//"") =~ m/^(..:..:..:..:..:..)(?:,(.*))?$/;
    unless ($addr) {
        logger::lsprintf("ERROR: usage: /connect XX:XX:XX:XX:XX:XX[,option=value]\n");
        logger::lsprintf("ERROR:   options: uart_at=0|1,security_level=none|low|medium|high|fips,\n");
        logger::lsprintf("ERROR:            io_capability=display-only|display-yesno|keyboard-only|no-input-output|keyboard-display,\n");
        logger::lsprintf("ERROR:            pin=NNNN,addr_type=public|random\n");
        logger::lsprintf("ERROR:   Note: Static random addresses must have MSB bits = 11 (first octet 0xC0-0xFF)\n");
        logger::lsprintf("ERROR: ARGUMENT: $tgt\n");
        return 0;
    }

    # Validate MAC address format using proper validation function
    unless (is_valid_ble_address($addr)) {
        logger::lsprintf("ERROR: Invalid MAC address format: $addr\n");
        return 0;
    }
    foreach my $t (@{$cfg->{targets}}) {
        if ($t->{b} eq $addr) {
            logger::lsprintf("ERROR: Already configured target: $addr\n");
            return 0;
        }
    }

    # Parse options, tokenize
    my @parsed_opts = split m/,/, $opts//"";
    my %parsed_opts;
    foreach my $o (@parsed_opts) {
        my ($k, $v) = ($o =~ m/^(.*?)=(.*)$/);
        if (defined $k and defined $v) {
            $parsed_opts{$k} = $v;
        } else {
            $parsed_opts{$o} = 1; # flag option
        }
    }

    # Validate and convert security_level option
    if (defined $parsed_opts{security_level}) {
        my $profile = lc($parsed_opts{security_level});
        if (defined(my $p = get_security_profile($profile))) {
            $parsed_opts{security_level} = $p;
        } else {
            logger::lsprintf("ERROR: Invalid security profile '$profile'. Valid options: none, low, medium, high, fips.\n");
            return 0;
        }
    }

    # Validate and convert io_capability option
    if (defined $parsed_opts{io_capability}) {
        my $io_cap = lc($parsed_opts{io_capability});
        if (defined(my $o = io_capability_name($io_cap))) {
            $parsed_opts{io_capability} = $o;
        } else {
            logger::lsprintf("ERROR: Invalid IO capability '$io_cap'. Valid options: display-only, display-yesno, keyboard-only, no-input-output, keyboard-display.\n");
            return 0;
        }
    }

    # Validate PIN format
    if (defined $parsed_opts{pin}) {
        unless ($parsed_opts{pin} =~ /^\d{4,6}$/) {
            logger::lsprintf("ERROR: Invalid PIN format. PIN must be 4-6 digits.\n");
            return 0;
        }
    }

    # Validate and set address type
    if (defined $parsed_opts{addr_type}) {
        my $type = lc($parsed_opts{addr_type});
        if ($type eq 'public') {
            $parsed_opts{addr_type} = 1; # BDADDR_LE_PUBLIC
        } elsif ($type eq 'random') {
            $parsed_opts{addr_type} = 2; # BDADDR_LE_RANDOM
            # Validate static random address format
            unless (is_valid_static_random_address($addr)) {
                logger::lsprintf("ERROR: Invalid static random address: $addr\n");
                logger::lsprintf("ERROR: Static random addresses must have the two most significant bits set to '11' (0xC0-0xFF in first octet)\n");
                return 0;
            }
        } else {
            logger::lsprintf("ERROR: Invalid address type '$type'. Valid options: public, random\n");
            return 0;
        }
    } else {
        # Auto-detect or use global public default
        $parsed_opts{addr_type} = detect_address_type($addr);
        if ($parsed_opts{addr_type} == 2) { # BDADDR_LE_RANDOM
            my $subtype = get_random_address_subtype($addr);
            logger::info("Using random address type for: $addr ($subtype)");
        } else {
            logger::info("Using public address type for: $addr");
        }
    }

    # info
    logger::info("Adding target: $addr (" . get_address_type_name($parsed_opts{addr_type}) . ") with options: " .
                 join(', ', map {
                     my $val = $parsed_opts{$_};
                     if ($_ eq 'addr_type') {
                         "$_=" . get_address_type_name($val);
                     } elsif (defined $val && $val ne '1') {
                         "$_=$val";
                     } else {
                         $_;
                     }
                 } grep { $_ ne 'addr_type' } keys %parsed_opts));

    # adding the target
    push @{$cfg->{targets}}, {
        b => $addr,
        l => \%parsed_opts
    };
    return 1;
}

our @cmds;
BEGIN {
    @cmds = qw(/exit /quit /disconnect /connect /help /debug /logging /loglevel /man /usage /switch /script /security /pair /unpair /primary /char-desc /info);
};

sub execute_at_script {
    my ($file) = @_;
    unless (-r $file) {
        logger::lsprintf("Cannot read script file: $file\n");
        return 1;
    }
    open(my $fh, '<', $file) or do {
        logger::lsprintf("Failed to open $file: $!\n");
        return 1;
    };
    logger::lsprintf("Executing script: $file\n");
    utils::load_cpan("Safe") or do {
        logger::lsprintf("Cannot load Safe module, cannot execute script.\n");
        close $fh;
        return 1;
    };
    my $r_code = 0;
    while (my $cmd = <$fh>) {
        chomp $cmd;
        $cmd =~ s/^\s+//; $cmd =~ s/\s+$//;
        next if $cmd eq '' || $cmd =~ /^#/;
        logger::lsprintf("> $cmd\n");

        # handle variable interpolation in a safe compartment
        eval {
            no warnings 'uninitialized';
            local $SIG{__DIE__}  =
            local $SIG{__WARN__} =
            local $SIG{WINCH}    =
            local $SIG{HUP}      =
            local $SIG{PIPE}     =
            local $SIG{INT}      =
            local $SIG{TERM}     = 'DEFAULT';
            my $sf = Safe->new();
            $sf->share_from('main', ['@ARGV']);
            unless($cmd =~ s/{(.*?)}/do {
                # allow code blocks
                my $r = $sf->reval($1);
                if($@){
                    chomp(my $err = $@);
                    logger::lsprintf("Script code error: $err\n");
                    $r_code = 1;
                    return;
                }
                unless(defined $cmd){
                    logger::lsprintf("Script code did not return a command to execute.\n");
                    $r_code = 1;
                    return;
                }
                $r;
                }/gemsx){
                $cmd =~ s/\\/\\\\/g; # escape slashes
                $cmd =~ s/"/\\"/g;   # escape quotes
                $cmd = $sf->reval("\"$cmd\"");
                if($@){
                    chomp(my $err = $@);
                    logger::lsprintf("Script variable interpolation error: $err\n");
                    $r_code = 1;
                    return;
                }
                unless(defined $cmd){
                    logger::lsprintf("Script variable interpolation resulted in undefined command.\n");
                    $r_code = 1;
                    return;
                }
            }
        };
        if($@){
            chomp(my $err = $@);
            logger::lsprintf("Script variable interpolation error: $err\n");
            $r_code = 1;
            next;
        }

        # now handle the command
        my $r = handle_command($cmd);
        if(!$r) {
            # data to be sent to the current connection
            logger::debug("Adding command to outbox:", $cmd);
            push @{$::OUTBOX}, "$cmd\n";
        }
    }
    close $fh;
    return $r_code;
}

sub handle_command {
    my ($line) = @_;
    chomp($line);
    $line =~ s/^\s+//; $line =~ s/\s+$//;
    return 0 if $line eq '' || $line =~ /^#/;

    logger::debug("command", $line);
    if ($line =~ m|^/exit| or $line =~ m|^/quit|) {
        $::DATA_LOOP_EXIT_WANTED = 1;
        return 1;
    }
    if ($line =~ m|^/man|) {
        utils::manpage(0);
        return 1;
    }
    if ($line =~ m|^/usage|) {
        utils::usage(-verbose => 1, -exitval => 'NOEXIT');
        return 1;
    }
    if ($line =~ m|^/connect\s*(\S+)?|) {
        main::add_target($::APP_OPTS, $1, 1);
        return 1;
    }
    if ($line =~ m|^/disconnect\s*(.*)|) {
        my $tgt = $1 // '';
        if(!@{$::APP_OPTS->{targets}}) {
            logger::lsprintf("No targets configured to disconnect.\n");
            return 1;
        }
        if($tgt){
            $tgt =~ s/^\s+|\s+$//g; # trim whitespace
            my $found = 0;
            foreach my $c (sort keys %{$::APP_CONN}) {
                if ($::APP_CONN->{$c}->{cfg}{b} eq $tgt) {
                    main::removing_tgt($::APP_CONN, $::APP_CONN->{$c});
                    $::COMMAND_BUFFER = undef;
                    $::CURRENT_CONNECTION = undef;
                    $found = 1;
                    last;
                }
            }
            foreach my $t (@{$::APP_OPTS->{targets}}) {
                if ($t->{b} eq $tgt) {
                    @{$::APP_OPTS->{targets}} = grep {$_->{b} ne $tgt} @{$::APP_OPTS->{targets}};
                    last;
                }
            }
            if (!$found) {
                logger::lsprintf("No target found with address: $tgt\n");
            } else {
                logger::lsprintf("Disconnected target: $tgt\n");
            }
            return 1;
        } else {
            $::COMMAND_BUFFER = undef;
            $::CURRENT_CONNECTION = undef;
            @{$::APP_OPTS->{targets}} = ();
            foreach my $c (sort keys %{$::APP_CONN}){
                (delete $::APP_CONN->{$c})->cleanup();
            }
            logger::lsprintf("Disconnected all BLE connections.\n");
        }
        return 1;
    }
    if ($line =~ m|^/script\s+(\S+)|) {
        return execute_at_script($1);
    }
    if ($line =~ m|^/debug\s*(on\|off)?|) {
        my $arg = $1 // '';
        if ($arg eq 'on') {
            utils::cfg('loglevel', 'DEBUG');
            utils::cfg('debug', 1);
            logger::lsprintf("Debugging enabled (loglevel=DEBUG)\n");
        } elsif ($arg eq 'off') {
            utils::cfg('loglevel', 'NONE');
            utils::cfg('debug', 0);
            logger::lsprintf("Debugging disabled (loglevel=INFO)\n");
        } else {
            logger::lsprintf("Usage: /debug on|off\n");
        }
        return 1;
    }
    if ($line =~ m|^/logging\s*(on\|off)?|) {
        my $arg = $1 // '';
        if ($arg eq 'on') {
            utils::cfg('loglevel', 'INFO');
            logger::lsprintf("Logging enabled (loglevel=INFO)\n");
        } elsif ($arg eq 'off') {
            utils::cfg('loglevel', 'NONE');
            logger::lsprintf("Logging disabled (loglevel=NONE)\n");
        } else {
            logger::lsprintf("Usage: /logging on|off\n");
        }
        return 1;
    }
    if ($line =~ m|^/loglevel\s*(none\|info\|warn\|error\|debug)?|) {
        my $lvl = $1;
        if (defined $lvl) {
            utils::cfg('loglevel', uc($lvl));
            logger::lsprintf("Log level set to $lvl\n");
        } else {
            logger::lsprintf("Usage: /loglevel <none|info|warn|error|debug>\n");
        }
        return 1;
    }
    if ($line =~ m|^/help|) {
        logger::lsprintf(join(", ", @main::cmds)."\n\n");
        return 1;
    }
    if ($line =~ m|^/switch\s*(\S+)?|) {
        my $tgt = $1 // '';
        if (!$tgt) {
            logger::lsprintf("Usage: /switch <BTADDR>\n");
            logger::lsprintf("Connected devices:\n");
            foreach my $c (values %{$::APP_CONN}) {
                logger::lsprintf("  $c->{cfg}{b}\n");
            }
            return 1;
        }
        my $found = 0;
        foreach my $c (values %{$::APP_CONN}) {
            if ($c->{cfg}{b} eq $tgt) {
                $::CURRENT_CONNECTION = $c;
                logger::lsprintf("Switched to device: $tgt\n");
                $found = 1;
                last;
            }
        }
        logger::lsprintf("No connected device with address: $tgt\n") unless $found;
        return 1;
    }
    if ($line =~ m|^/security\s*(.*)|) {
        my $arg = $1 // '';
        if ($arg eq '') {
            # Display current security settings
            logger::lsprintf("Current security settings:\n");
            if ($::CURRENT_CONNECTION) {
                my $cfg = $::CURRENT_CONNECTION->{cfg};
                my %security_names = (
                    0 => 'none',    # BT_SECURITY_SDP
                    1 => 'low',     # BT_SECURITY_LOW
                    2 => 'medium',  # BT_SECURITY_MEDIUM
                    3 => 'high',    # BT_SECURITY_HIGH
                    4 => 'fips',    # BT_SECURITY_FIPS
                );
                my %io_cap_names = (
                    0 => 'display-only',     # BT_IO_CAP_DISPLAY_ONLY
                    1 => 'display-yesno',    # BT_IO_CAP_DISPLAY_YESNO
                    2 => 'keyboard-only',    # BT_IO_CAP_KEYBOARD_ONLY
                    3 => 'no-input-output',  # BT_IO_CAP_NO_INPUT_OUTPUT
                    4 => 'keyboard-display', # BT_IO_CAP_KEYBOARD_DISPLAY
                );
                my $sec_level = $cfg->{l}{security_level} // $::APP_OPTS->{_security_level} // 1; # BT_SECURITY_LOW
                my $io_cap = $cfg->{l}{io_capability} // $::APP_OPTS->{_io_capability} // 3; # BT_IO_CAP_NO_INPUT_OUTPUT
                logger::lsprintf("  Device: $cfg->{b}\n");
                logger::lsprintf("  Security Level: " . ($security_names{$sec_level} // $sec_level) . "\n");
                logger::lsprintf("  IO Capability: " . ($io_cap_names{$io_cap} // $io_cap) . "\n");
                logger::lsprintf("  PIN: " . (defined $cfg->{l}{pin} ? "****" : "not set") . "\n");
            } else {
                logger::lsprintf("  Global Security Level: " . ($::APP_OPTS->{_security_level} // 1) . "\n"); # BT_SECURITY_LOW
                logger::lsprintf("  Global IO Capability: " . ($::APP_OPTS->{_io_capability} // 3) . "\n"); # BT_IO_CAP_NO_INPUT_OUTPUT
                logger::lsprintf("  Global PIN: " . (defined $::APP_OPTS->{_pin} ? "****" : "not set") . "\n");
                logger::lsprintf("  No current connection to show device-specific settings.\n");
            }
        } else {
            logger::lsprintf("Usage: /security (shows current security settings)\n");
            logger::lsprintf("To change security settings, use /connect with security options\n");
        }
        return 1;
    }
    if ($line =~ m|^/pair\s*(\S+)?\s*(\d{4,6})?|) {
        my ($tgt, $pin) = ($1, $2);
        if (!$tgt && !$::CURRENT_CONNECTION) {
            logger::lsprintf("Usage: /pair [BTADDR] [PIN]\n");
            logger::lsprintf("No current connection. Specify a Bluetooth address.\n");
            return 1;
        }

        my $target_conn = $::CURRENT_CONNECTION;
        if ($tgt) {
            $target_conn = undef;
            foreach my $c (values %{$::APP_CONN}) {
                if ($c->{cfg}{b} eq $tgt) {
                    $target_conn = $c;
                    last;
                }
            }
            if (!$target_conn) {
                logger::lsprintf("Device $tgt not connected. Connect first with /connect\n");
                return 1;
            }
        }

        logger::lsprintf("Initiating pairing with device: $target_conn->{cfg}{b}\n");
        if ($pin) {
            logger::lsprintf("Using provided PIN: ****\n");
            $target_conn->{cfg}{l}{pin} = $pin;
        } elsif ($target_conn->{cfg}{l}{pin}) {
            logger::lsprintf("Using configured PIN: ****\n");
        } else {
            logger::lsprintf("No PIN provided - using device default pairing method\n");
        }

        # Note: Actual pairing would require additional Bluetooth management
        # This is a placeholder for pairing initiation
        logger::lsprintf("Pairing request sent. Check device for confirmation prompts.\n");
        return 1;
    }
    if ($line =~ m|^/primary\s*(\S+)?\s*(\S+)?|) {
        my ($start_handle, $end_handle) = ($1, $2);

        if (!$::CURRENT_CONNECTION) {
            logger::lsprintf("No current connection. Connect to a device first with /connect\n");
            return 1;
        }

        # Parse handle arguments (gatttool format: optional start and end handles)
        $start_handle = hex($start_handle) if ($start_handle//"") =~ /^0x/i;
        $end_handle   = hex($end_handle)   if ($end_handle//"")   =~ /^0x/i;

        $start_handle //= 0x0001;  # Default start handle
        $end_handle   //= 0xFFFF;  # Default end handle

        # Validate handle range
        if ($start_handle < 1 || $start_handle > 0xFFFF) {
            logger::lsprintf("Invalid start handle. Must be 0x0001-0xFFFF\n");
            return 1;
        }
        if ($end_handle < $start_handle || $end_handle > 0xFFFF) {
            logger::lsprintf("Invalid end handle. Must be >= start handle and <= 0xFFFF\n");
            return 1;
        }

        logger::lsprintf("Starting primary service discovery (0x%04X - 0x%04X)\n", $start_handle, $end_handle);

        # Store the discovery state for this request
        $::CURRENT_CONNECTION->{_primary_discovery_start}  = $start_handle;
        $::CURRENT_CONNECTION->{_primary_discovery_end}    = $end_handle;
        $::CURRENT_CONNECTION->{_primary_discovery_active} = 1;

        # Send the primary service discovery request
        $::CURRENT_CONNECTION->{_outbuffer} .= ble::gatt_discovery_primary($start_handle, $end_handle);

        return 1;
    }
    if ($line =~ m|^/char-desc\s*(\S+)?\s*(\S+)?|) {
        my ($start_handle, $end_handle) = ($1, $2);

        if (!$::CURRENT_CONNECTION) {
            logger::lsprintf("No current connection. Connect to a device first with /connect\n");
            return 1;
        }

        # Parse handle arguments (gatttool format: optional start and end handles)
        $start_handle = hex($start_handle) if ($start_handle//"") =~ /^0x/i;
        $end_handle   = hex($end_handle)   if ($end_handle//"")   =~ /^0x/i;

        $start_handle //= 0x0001;  # Default start handle
        $end_handle   //= 0xFFFF;  # Default end handle

        # Validate handle range
        if ($start_handle < 1 || $start_handle > 0xFFFF) {
            logger::lsprintf("Invalid start handle. Must be 0x0001-0xFFFF\n");
            return 1;
        }
        if ($end_handle < $start_handle || $end_handle > 0xFFFF) {
            logger::lsprintf("Invalid end handle. Must be >= start handle and <= 0xFFFF\n");
            return 1;
        }

        logger::lsprintf("Starting characteristic descriptor discovery (0x%04X - 0x%04X)\n", $start_handle, $end_handle);

        # Store the discovery state for this request
        $::CURRENT_CONNECTION->{_char_desc_discovery_start}  = $start_handle;
        $::CURRENT_CONNECTION->{_char_desc_discovery_end}    = $end_handle;
        $::CURRENT_CONNECTION->{_char_desc_discovery_active} = 1;

        # Send the descriptor discovery request (Find Information Request)
        $::CURRENT_CONNECTION->{_outbuffer} .= ble::gatt_desc_discovery($start_handle, $end_handle);

        return 1;
    }
    if ($line =~ m|^/unpair\s*(\S+)?|) {
        my $tgt = $1;
        if (!$tgt && !$::CURRENT_CONNECTION) {
            logger::lsprintf("Usage: /unpair [BTADDR]\n");
            logger::lsprintf("No current connection. Specify a Bluetooth address.\n");
            return 1;
        }

        my $target_addr = $tgt || $::CURRENT_CONNECTION->{cfg}{b};
        logger::lsprintf("Unpairing device: $target_addr\n");

        # Note: Actual unpairing would require additional Bluetooth management
        # This is a placeholder for unpairing
        logger::lsprintf("Unpair request sent for device: $target_addr\n");
        return 1;
    }
    if ($line =~ m|^/info|) {
        if (!$::CURRENT_CONNECTION) {
            logger::lsprintf("No current connection. Connect to a device first with /connect\n");
            return 1;
        }
        # Send a Read By Type Request for the Device Name characteristic UUID (use 16-bit form)
        $::CURRENT_CONNECTION->{_info_request_active} = sub {
            my ($self, $data) = @_;
            delete $self->{_info_request_active}; # clear the state

            # Parse the Read By Type Response for Device Name
            my ($len) = unpack('xC', $data);
            if (($len//0) < 3) {
                logger::lsprintf("Device Name: <Not found or error reading>\n");
                return;
            }

            # The response format is: opcode(1) length(1) [handle(2) value(N)]...
            # For Device Name, we expect the value to be a string
            my ($handle, $value) = unpack('S<a*', substr($data, 2, $len));
            if (length($value//"") > 0) {
                # Convert the value to a readable string
                my $device_name = $value;
                $device_name =~ s/\0+$//; # Remove null terminators
                logger::lsprintf("Device Name: $device_name\n");
            } else {
                logger::lsprintf("Device Name: <Empty or unavailable>\n");
            }
            return;
        };
        $::CURRENT_CONNECTION->{_outbuffer} .= ble::gatt_read_by_type(0x0001, 0xFFFF, 0x2A00);
        return 1;
    }
    if ($line =~ m|^/|) {
        logger::lsprintf("Unknown command: $line\n");
        return 1;
    }
    return;
}


BEGIN {
package f;
# basically Fcntl, but less memory hungry
*F_GETFD     = sub (){0x0001};
*O_RDWR      = sub (){0x0002};
*F_GETFL     = sub (){0x0003};
*F_SETFL     = sub (){0x0004};
*O_NONBLOCK  = sub (){0x0800};

package e;
# basically Errno, but less memory hungry
*EINTR       = sub (){   4};
*EAGAIN      = sub (){  11};
*EINPROGRESS = sub (){ 115};
};


package input::tty;

use strict;

our $_term;
our $BASE_DIR;
our $HISTORY_FILE;

BEGIN {
    $BASE_DIR     //= $ENV{BLE_UART_DIR}
                  // ($ENV{HOME}//"/home/$ENV{LOGNAME}")."/.ble_uart";
    $HISTORY_FILE //= $ENV{BLE_UART_HISTORY_FILE}
                  // "${BASE_DIR}_history";
};

BEGIN {
package colors;

our $red_color           = "\033[0;31m";
our $bright_red_color    = "\033[38;5;196;1m";
our $green_color         = "\033[0;32m";
our $dark_green_color    = "\033[38;5;28;1m";
our $yellow_color1       = "\033[0;33m";
our $dark_yellow_color   = "\033[38;5;136;1m";
our $bright_yellow_color = "\033[38;5;226;1m";
our $blue_color1         = "\033[0;34m";
our $bright_blue_color1  = "\033[38;5;21;1m";
our $blue_color2         = "\033[38;5;25;1m";
our $bright_blue_color2  = "\033[38;5;33;1m";
our $blue_color3         = "\033[38;5;13;1m";
our $bright_blue_color3  = "\033[38;5;39;1m";
our $magenta_color       = "\033[0;35m";
our $cyan_color          = "\033[0;36m";
our $white_color         = "\033[0;37m";
our $reset_color         = "\033[0m";
};

sub new {
    my ($class, $cfg) = @_;
    return $_term if defined $_term;
    $cfg //= {};
    my $self = bless {%$cfg}, ref($class)||$class;

    # if we started up with PERL_SKIP_LOCALE_INIT=1 or LC_ALL=C, perl won't do
    # setlocale() and we don't use memory, however, if we still use a color
    # term and utf8, we need to setlocale() ourselves for readline to work
    # properly with the window width detection. This uses MORE memory as we
    # need POSIX for that.
    if(($ENV{LC_ALL}//"C") eq "C"){
        require POSIX;
        POSIX::setlocale(POSIX::LC_ALL(), "en_US.UTF-8");
    }
    local $ENV{PERL_RL}   = 'Gnu';
    local $ENV{TERM}      = $ENV{TERM} // 'vt220';
    local $ENV{COLORTERM} = $ENV{COLORTERM} // 'truecolor';
    my $sig_h = $SIG{INT};
    eval {require Term::ReadLine; require Term::ReadLine::Gnu};
    if($@){
        logger::error("Please install Term::ReadLine and Term::ReadLine::Gnu\n\nE.g.:\n  sudo apt install libterm-readline-gnu-perl");
        exit 1;
    }
    $Term::ReadLine::Gnu::Attribs{attempted_completion_function} = \&chat_word_completions_cli;
    $Term::ReadLine::Gnu::Attribs{ignore_completion_duplicates} = 1;
    $Term::ReadLine::Gnu::Attribs{horizontal_scroll_mode} = 0;
    $Term::ReadLine::Gnu::Attribs{catch_signals} = 0;
    $Term::ReadLine::Gnu::Attribs{catch_sigwinch} = 0;
    my $term = Term::ReadLine->new("aicli");
    $term->read_init_file("$BASE_DIR/inputrc");
    $term->ReadLine('Term::ReadLine::Gnu') eq 'Term::ReadLine::Gnu'
        or die "Term::ReadLine::Gnu is required\n";

    $term->using_history();
    $term->ReadHistory($HISTORY_FILE);
    $term->clear_signals();
    $SIG{WINCH} = sub {
        $term->reset_screen_size();
        my ($n_rows, $n_cols) = $term->get_screen_size();
        $term->redisplay();
        logger::debug("Terminal resized, redisplay, new size: ", $n_cols, "x", $n_rows);
        return;
    };
    $SIG{INT} = $SIG{TERM} = sub {
        $term->write_history($HISTORY_FILE);
        $term->clear_message();
        $term->crlf();
        $term->set_prompt("");
        $term->redisplay();
        $term->resize_terminal();
        $term->free_line_state();
        $term->cleanup_after_signal();
        return &$sig_h(@_) if defined $sig_h and ref($sig_h) eq 'CODE';
        return;
    };
    my ($t_ps1, $t_ps2) = $self->create_prompt();
    $term->callback_handler_install(
        $t_ps1,
        sub {
            my ($n_ps1, $n_ps2) = $self->create_prompt();
            $self->rl_cb_handler($n_ps1, $n_ps2, @_);
            return;
        }
    );
    my ($n_rows, $n_cols) = $term->get_screen_size();
    logger::debug("Terminal size: ", $n_cols, "x", $n_rows);
    $term->save_prompt();
    $term->clear_message();
    $term->message(($::APP_OPTS->{_color_ok}?$colors::bright_red_color:"")."Welcome to the BLE UART CLI".($::APP_OPTS->{_color_ok}?$colors::reset_color:""));
    $term->crlf();
    $term->restore_prompt();
    $term->save_prompt();
    $term->clear_message();
    $term->message(($::APP_OPTS->{_color_ok}?$colors::bright_yellow_color:"")."Type /help for available commands, /usage for usage doc".($::APP_OPTS->{_color_ok}?$colors::reset_color:""));
    $term->crlf();
    $term->restore_prompt();
    $term->on_new_line();
    $term->redisplay();
    $term->reset_screen_size();
    $self->{_rl} = $term;
    logger::log_printer(sub {
        my ($lmsg) = @_;
        no warnings;
        my ($n_ps1, $ps2) = $self->create_prompt();
        my $t = $self->{_rl};
        $t->set_prompt($n_ps1);
        $t->save_prompt();
        $t->clear_message();
        $t->message($lmsg);
        $t->restore_prompt();
        $t->on_new_line();
        $t->redisplay();
        return;
    });
    return $_term = $self;
}

sub infd {
    my ($self) = @_;
    return fileno($self->{_rl}->IN());
}

sub infh {
    my ($self) = @_;
    return $self->{_rl}->IN();
}

sub outfd {
    my ($self) = @_;
    return fileno($self->{_rl}->OUT());
}

sub outfh {
    my ($self) = @_;
    return $self->{_rl}->OUT();
}

sub do_read {
    my ($self) = @_;
    logger::debug(">>TTY>> waiting for input from TTY");
    return $self->{_rl}->callback_read_char();
}

sub show_message {
    my ($self, $m) = @_;
    my ($n_rows, $n_cols) = $self->{_rl}->get_screen_size();
    return if !$n_cols;
    my ($n_ps1, $ps2) = $self->create_prompt();
    my $t = $self->{_rl};
    $t->set_prompt($n_ps1);
    $t->save_prompt();
    $t->clear_message();
    $t->message($m);
    $t->crlf();
    $t->restore_prompt();
    $t->on_new_line();
    $t->redisplay();
    return;
}

sub spin {
    my ($self) = @_;
    $self->{_spinners}  //= $::APP_OPTS->{_utf8_ok} ? [qw(⠋ ⠙ ⠹ ⠸ ⠼ ⠴ ⠦ ⠧)] : [qw(- \ | /)];
    $self->{_spin_pos}  //= 0;
    $self->{_spin_icon} //= do {
        my $s_icon;
        if($::APP_OPTS->{_color_ok}){
            $s_icon = $::APP_OPTS->{_utf8_ok}
            ? " ".$colors::red_color.'⌛'.$colors::bright_red_color." Waiting for response...".$colors::reset_color
            : $colors::bright_red_color." Waiting for response...".$colors::reset_color;
        } else {
            $s_icon = $::APP_OPTS->{_utf8_ok}
            ? " ⌛ Waiting for response..."
            : " Waiting for response...";
        }
        $s_icon;
    };
    $self->{_spinner_color} = $::APP_OPTS->{_color_ok} ? $colors::bright_red_color : "";
    my $t = $self->{_rl};
    $t->save_prompt();
    $t->clear_message();
    $t->message($self->{_spinner_color}.$self->{_spinners}[$self->{_spin_pos}].$self->{_spin_icon});
    $t->restore_prompt();
    $t->on_new_line_with_prompt();
    $self->{_spin_pos}++;
    $self->{_spin_pos} = 0 if $self->{_spin_pos} >= @{$self->{_spinners}};
    return;
}

sub chat_word_completions_cli {
    my ($text, $line, $start, $end) = @_;
    $line =~ s/ +$//g;
    my @rcs = ();
    my @wrd = split m/\s+/, $line, -1;
    logger::debug("W: >", @wrd, "<\n");
    foreach my $w (@wrd) {
        next unless $w =~ m|^/|;
        foreach my $k (@main::cmds) {
            push @rcs, $k if !index($k, $w) or $k eq $w;
        }
    }
    logger::debug("R: >", @rcs, "<");
    return '', @rcs;
}

sub cleanup {
    my ($self) = @_;
    my $t = $self->{_rl};
    return unless defined $t;
    $t->callback_handler_remove();
    $t->cleanup_after_signal();
    $t->callback_handler_remove();
    $t->write_history($HISTORY_FILE);
    $t->clear_message();
    $t->crlf();
    $t->set_prompt("");
    $t->redisplay();
    $t->cleanup_after_signal();
    return;
}

sub rl_cb_handler {
    my ($self, $t_ps1, $t_ps2, $line) = @_;
    my $t = $self->{_rl};
    if(!defined $line){
        $t->callback_handler_remove();
        $::DATA_LOOP = 0; # exit the main loop
        return;
    }
    $t->set_prompt($t_ps1);
    my $buf = \($self->{_buf} //= '');
    if($line !~ m/^$/ms){
        $line =~ s/^\s+//;
        $line =~ s/\s+$//;
        $line =~ s/\r?\n$//;
        logger::debug(">>TTY>>", length($line), " bytes read from TTY: $line");
        # not a command it OR we already had a buffer,
        if(utils::cfg('interactive_multiline', 0)){
            # multiline input, we need to buffer it

            # do readline stuff
            $t->set_prompt($t_ps2);

            # add to buffer until we have an empty line entered
            $$buf .= "$line\n";
            return;
        } else {
            # just process the line
            logger::debug(">>TTY>> processing line:", $line);

            # do readline stuff
            $t->addhistory($line);
            $t->WriteHistory($HISTORY_FILE);
            $t->set_prompt($t_ps1);
            $t->on_new_line_with_prompt();
            $t->redisplay();

            # add to the output buffer
            $$buf .= "$line\n";
            push @{$::OUTBOX}, $$buf;
            $$buf = '';
            return;
        }
        return;
    } else {
        logger::debug(">>TTY>> empty line read from TTY, processing buffer");
        # empty line, this is command execution if we have a buffer
        if(length($$buf)){
            logger::debug("BUF: >>", $$buf, "<<");
            $t->addhistory($$buf);
            $t->WriteHistory($HISTORY_FILE);
            $t->set_prompt($t_ps1);
            $t->on_new_line_with_prompt();
            $t->redisplay();
            push @{$::OUTBOX}, $$buf;
            $$buf = '';
        }
        return;
    }
    return;
}

sub create_prompt {
    my ($self) = @_;
    # https://jafrog.com/2013/11/23/colors-in-terminal.html
    # https://ss64.com/bash/syntax-colors.html
    # If a BLE device is connected, show its address in yellow
    my $ble_addr = $::CURRENT_CONNECTION ? $::CURRENT_CONNECTION->{cfg}{b} : undef;
    my $PR = "";
    if($ble_addr){
        $PR = $::APP_OPTS->{_utf8_ok} ? "❲$ble_addr❳" : "|$ble_addr|";
        if($::APP_OPTS->{_color_ok}) {
            $PR  = $colors::yellow_color1.$PR.$colors::reset_color;
            $PR .= $colors::bright_blue_color2."AT".$colors::reset_color;
        } else {
            $PR .= "AT";
        }
    }
    my $PP1 = $::APP_OPTS->{_utf8_ok} ? "► " : "> ";
    my $PP2 = $::APP_OPTS->{_utf8_ok} ? "│ " : "| ";
    if($::APP_OPTS->{_color_ok}) {
        $PP1 = $colors::bright_blue_color2.$PP1.$colors::reset_color;
        $PP2 = $colors::bright_blue_color3.$PP2.$colors::reset_color;
    }
    my $prompt_term1 = "$PR$PP1";
    my $prompt_term2 = "$PP2";
    if ($::APP_OPTS->{_color_ok}) {
        if ($ble_addr) {
            $prompt_term1 = $colors::green_color.$prompt_term1.$colors::reset_color;
        } else {
            $prompt_term1 = $colors::blue_color3.$prompt_term1.$colors::reset_color;
        }
        $prompt_term2 = $colors::blue_color3.$prompt_term2.$colors::reset_color;
    }
    # should be "Safe", as no user input is used in the prompt
    my $ps1 = eval "return \"$prompt_term1\"" || $PP1;
    my $ps2 = eval "return \"$prompt_term2\"" || $PP2;
    return ($ps1, $ps2);
}

sub DESTROY {
    my ($self) = @_;
    $self->cleanup();
    return;
}


package input::stdin;

use strict;

sub new {
    my ($class, $cfg) = @_;
    $cfg //= {};
    my $self = bless {%$cfg}, ref($class)||$class;

    # Set STDIN to non-blocking mode
    my $flags = fcntl(STDIN, f::F_GETFL, 0)
        or die "Can't get flags for STDIN: $!\n";
    fcntl(STDIN, f::F_SETFL, $flags | f::O_NONBLOCK)
        or die "Can't set STDIN non-blocking: $!\n";

    $self->{_buffer} = "";
    return $self;
}

sub infd {
    my ($self) = @_;
    return fileno(STDIN);
}

sub infh {
    my ($self) = @_;
    return \*STDIN;
}

sub outfd {
    my ($self) = @_;
    return fileno(STDOUT);
}

sub outfh {
    my ($self) = @_;
    return \*STDOUT;
}

sub do_read {
    my ($self) = @_;
    my $r = sysread(STDIN, my $data, 1);
    if (!defined $r) {
        return if $! == e::EAGAIN;
        die "Error reading from STDIN: $!\n";
    } elsif ($r == 0) {
        # EOF - signal main loop to exit
        $::DATA_LOOP_EXIT_WANTED = 0;
        return;
    }
    $self->{_buffer} .= $data;

    # Process complete lines
    while ($self->{_buffer} =~ s/^([^\r\n]*)\r?\n//) {
        my $line = $1;
        chomp $line;
        $line =~ s/^\s+//;
        $line =~ s/\s+$//;

        if (length($line) > 0) {
            logger::debug(">>STDIN>> processing line >>$line<<");
            push @{$::OUTBOX}, "$line\n";
        }
    }
    return 1;
}

sub show_message {
    my ($self, $m) = @_;
    return unless length($m//"");
    no warnings 'utf8'; # disable utf8 warnings for STDIN mode
    print STDOUT $m;
    return;
}

sub spin {
    my ($self) = @_;
    # No spinner for STDIN mode
    return;
}

sub cleanup {
    my ($self) = @_;
    # Nothing special to clean up for STDIN
    return;
}


package ble;

use strict;

# constants for BLUETOOTH that come from bluez
BEGIN {
*SOL_SOCKET         = sub (){  1};
*SOCK_STREAM        = sub (){  1};
*SOCK_DGRAM         = sub (){  2};
*SOCK_RAW           = sub (){  3};
*SOCK_SEQPACKET     = sub (){  5};

*SO_ERROR           = sub (){  4};

*SOL_RAW            = sub (){255};
*SOL_PACKET         = sub (){263};

# Bluetooth address format
*AF_BLUETOOTH       = sub (){31};
*PF_BLUETOOTH       = sub (){31};

# Bluetooth socket types
*BT_SECURITY        = sub (){ 4};
*BT_SECURITY_SDP    = sub (){ 0};
*BT_SECURITY_LOW    = sub (){ 1};
*BT_SECURITY_MEDIUM = sub (){ 2};
*BT_SECURITY_HIGH   = sub (){ 3};
*BT_SECURITY_FIPS   = sub (){ 4};

# L2CAP constants
*BT_SNDMTU          = sub (){12};
*BT_RCVMTU          = sub (){13};

# Bluetooth Protocols
*BTPROTO_L2CAP      = sub (){ 0};
*BTPROTO_HCI        = sub (){ 1};
*BTPROTO_SCO        = sub (){ 2};
*BTPROTO_RFCOMM     = sub (){ 3};
*BTPROTO_BNEP       = sub (){ 4};
*BTPROTO_CMTP       = sub (){ 5};
*BTPROTO_HIDP       = sub (){ 6};
*BTPROTO_AVDTP      = sub (){ 7};

# Bluetooth Socket Options
*SOL_HCI            = sub (){ 0};
*SOL_L2CAP          = sub (){ 6};
*SOL_SCO            = sub (){17};
*SOL_RFCOMM         = sub (){18};

# Bluetooth Socket Options
*SOL_BLUETOOTH      = sub (){274};
*BDADDR_BREDR       = sub (){0x00};
*BDADDR_LE_PUBLIC   = sub (){0x01};
*BDADDR_LE_RANDOM   = sub (){0x02};
*BDADDR_ANY         = sub (){"\0\0\0\0\0\0"};
*BDADDR_ALL         = sub (){"\255\255\255\255\255\255"};
*BDADDR_LOCAL       = sub (){"\0\0\0\255\255\255"};

# BLE Security and Pairing constants
*BT_IO_CAP_DISPLAY_ONLY     = sub (){0x00};
*BT_IO_CAP_DISPLAY_YESNO    = sub (){0x01};
*BT_IO_CAP_KEYBOARD_ONLY    = sub (){0x02};
*BT_IO_CAP_NO_INPUT_OUTPUT  = sub (){0x03};
*BT_IO_CAP_KEYBOARD_DISPLAY = sub (){0x04};
*BT_POWER                   = sub (){0x09};
*BT_IO_CAP                  = sub (){0x0C};

# SMP (Security Manager Protocol) constants for pairing
*SMP_LTK_SLAVE              = sub (){0x01};
*SMP_LTK_MASTER             = sub (){0x02};
*SMP_IRK                    = sub (){0x03};
*SMP_CSRK_SLAVE             = sub (){0x04};
*SMP_CSRK_MASTER            = sub (){0x05};

# L2CAP constants
*L2CAP_OPTIONS              = sub (){0x01};
*L2CAP_CID_ATT              = sub (){0x04};
*L2CAP_CID_SIG              = sub (){0x05};
*L2CAP_PSM_SDP              = sub (){0x01};
};

our %err_msg;
BEGIN {
%err_msg = (
    0x01 => "Invalid Handle",
    0x02 => "Read Not Permitted",
    0x03 => "Write Not Permitted",
    0x04 => "Invalid PDU",
    0x05 => "Insufficient Authentication",
    0x06 => "Request Not Supported",
    0x07 => "Invalid Offset",
    0x08 => "Insufficient Authorization",
    0x09 => "Prepare Queue Full",
    0x0A => "Attribute Not Found",
    0x0B => "Attribute Not Long",
    0x0C => "Insufficient Encryption Key Size",
    0x0D => "Invalid Attribute Value Length",
    0x0E => "Unlikely Error",
    0x0F => "Insufficient Encryption",
    0x10 => "Unsupported Group Type",
    0x11 => "Insufficient Resources",
    0x12 => "Application Error",
    0x13 => "Attribute Not Found (GATT)",
    0x14 => "Attribute Not Long (GATT)",
);
};

sub new {
    my ($class, $cfg) = @_;
    return bless {_outbuffer => "", _outboxbuffer => "", _gatt_state => "mtu", cfg => $cfg}, ref($class)||$class;
}

sub init {
    my ($self, $blocking) = @_;
    $self->{_log_info} = "[".($self->{cfg}{b}||"no_bt")."]";
    logger::info("Initializing BLE uart handler for $self->{_log_info}");
    my ($r_btaddr, $l_btaddr) = ($self->{cfg}{b}, $self->{cfg}{l}{bt_listen_addr});
    if($l_btaddr){
        # check if the local bluetooth address is valid
        unless (is_valid_ble_address($l_btaddr)) {
            die "Invalid local bluetooth address: $l_btaddr\n";
        }
        # pack the local bluetooth address
        $l_btaddr = lc($l_btaddr);
        $l_btaddr =~ s/://g; # remove colons
        $l_btaddr = pack("H12", $l_btaddr);
        $l_btaddr = reverse $l_btaddr; # reverse the byte order
    } else {
        $l_btaddr = BDADDR_ANY; # use any local bluetooth address
    }
    my $l_addr = pack_sockaddr_bt(bt_aton($l_btaddr), 0);

    # new sockets, bind and request the right type for our bluetooth BLE UART
    # connection
    socket(my $s, AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP)
        // die "socket create problem: $!\n";
    my $fd = fileno($s);
    fcntl($s, f::F_GETFL, my $fcntl_flags = 0)
        // die "socket fcntl get problem: $!\n";
    $fcntl_flags |= f::O_RDWR; # set to read/write mode
    if($blocking){
        # set to blocking mode
        $fcntl_flags &= ~f::O_NONBLOCK;
    } else {
        # set to non-blocking mode
        $fcntl_flags |= f::O_NONBLOCK;
    }

    # set to non blocking mode now, and binmode
    my $cn_info = "$self->{_log_info} (fd: $fd)";
    fcntl($s, f::F_SETFL, $fcntl_flags)
        // die "socket non-blocking set problem $cn_info: $!\n";
    binmode($s)
        // die "binmode problem $cn_info: $!\n";

    # bind
    bind($s, $l_addr)
        // die "bind error $cn_info: $!\n";

    # Set security level based on configuration
    my $security_level = $self->{cfg}{l}{security_level} // $::APP_OPTS->{_security_level} // BT_SECURITY_LOW;
    setsockopt($s, SOL_BLUETOOTH, BT_SECURITY, pack("S", $security_level))
        // die "setsockopt BT_SECURITY problem $cn_info: $!\n";
    logger::debug("BLE Security level set to: $security_level");

    # Set IO capability for pairing
    my $io_capability = $self->{cfg}{l}{io_capability} // $::APP_OPTS->{_io_capability} // BT_IO_CAP_NO_INPUT_OUTPUT;
    setsockopt($s, SOL_BLUETOOTH, BT_IO_CAP, pack("S", $io_capability))
        // logger::debug("setsockopt BT_IO_CAP not supported or failed: $!");
    logger::debug("BLE IO capability set to: $io_capability");

    setsockopt($s, SOL_BLUETOOTH, BT_RCVMTU, pack("CC", 0xA0, 0x02))
        // logger::error("setsockopt BT_RCVMTU problem: $!");

    # now connect
    my $addr_type = $self->{cfg}{l}{addr_type} // BDADDR_LE_PUBLIC;
    my $r_addr = pack_sockaddr_bt(bt_aton($r_btaddr), 0, L2CAP_CID_ATT, $addr_type);
    if ($addr_type == BDADDR_LE_RANDOM) {
        my $subtype = main::get_random_address_subtype($r_btaddr);
        logger::info("Connecting to $r_btaddr with random address type ($subtype)");
    } else {
        logger::info("Connecting to $r_btaddr with public address type");
    }
    connect($s, $r_addr)
        // ($! == e::EINTR or $! == e::EAGAIN or $! == e::EINPROGRESS)
        or die "problem connecting to $cn_info: $!\n";

    # double check for socket errors
    my $err = getsockopt($s, ble::SOL_SOCKET(), ble::SO_ERROR());
    if(!defined $err){
        logger::error("Error getting socket options: $!");
    } else {
        $! = unpack("I", $err);
        if($!){
            logger::error("Socket error on $self->{_log_info} (fd: $fd): $!");
            return;
        }
    }

    # return info
    $self->{_nok}    = 1 if !$blocking;
    $self->{_socket} = $s;
    $self->{_fd}     = $fd;
    return $fd;
}

sub is_connected {
    my ($self) = @_;
    return unless $self->{_socket} and ($self->{_fd}//0) >= 0 and !exists $self->{_nok};
}

sub blocking {
    my ($self, $blocking) = @_;
    return unless defined $self->{_socket};
    return unless defined $blocking;

    fcntl($self->{_socket}, f::F_GETFL, my $fcntl_flags = 0)
        // die "socket fcntl get problem: $!\n";
    if($blocking){
        # set to blocking mode
        $fcntl_flags &= ~f::O_NONBLOCK;
    } else {
        # set to non-blocking mode
        $fcntl_flags |= f::O_NONBLOCK;
    }
    fcntl($self->{_socket}, f::F_SETFL, $fcntl_flags)
        // die "socket non-blocking set problem: $!\n";
    return;
}

# see https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/attribute-protocol--att-.html

# ATT Opcode name helper for debugging
sub get_att_opcode_name {
    my ($opcode) = @_;
    my %att_opcodes = (
        0x01 => "Error Response",
        0x02 => "Exchange MTU Request",
        0x03 => "Exchange MTU Response",
        0x04 => "Find Information Request",
        0x05 => "Find Information Response",
        0x06 => "Find By Type Value Request",
        0x07 => "Find By Type Value Response",
        0x08 => "Read By Type Request",
        0x09 => "Read By Type Response",
        0x0A => "Read Request",
        0x0B => "Read Response",
        0x0C => "Read Blob Request",
        0x0D => "Read Blob Response",
        0x0E => "Read Multiple Request",
        0x0F => "Read Multiple Response",
        0x10 => "Read By Group Type Request",
        0x11 => "Read By Group Type Response",
        0x12 => "Write Request",
        0x13 => "Write Response",
        0x16 => "Prepare Write Request",
        0x17 => "Prepare Write Response",
        0x18 => "Execute Write Request",
        0x19 => "Execute Write Response",
        0x1B => "Handle Value Notification",
        0x1D => "Handle Value Indication",
        0x1E => "Handle Value Confirmation",
        0x20 => "Read Multiple Variable Request",
        0x21 => "Read Multiple Variable Response",
        0x23 => "Multiple Handle Value Notification",
        0x52 => "Write Command",
        0xD2 => "Signed Write Command",
    );

    return $att_opcodes{$opcode} // sprintf("Unknown ATT Opcode (0x%02X)", $opcode);
}

# GATT Primary Service Discovery (ATT Read By Group Type Request)
sub gatt_discovery_primary {
    my ($start_handle, $end_handle) = @_;
    $start_handle //= 0x0001;
    $end_handle   //= 0xFFFF;
    my $uuid = pack("S<", 0x2800); # 16-bit UUID for Primary Service
    return pack("CS<S<a*", 0x10, $start_handle, $end_handle, $uuid);
}

# GATT Secondary Service Discovery (ATT Read By Group Type Request)
sub gatt_discovery_secondary {
    my ($start_handle, $end_handle) = @_;
    $start_handle //= 0x0001;
    $end_handle   //= 0xFFFF;
    my $uuid = pack("S<", 0x2801); # 16-bit UUID for Secondary Service
    return pack("CS<S<a*", 0x10, $start_handle, $end_handle, $uuid);
}

# GATT Characteristic Discovery (ATT Read By Type Request)
sub gatt_char_discovery {
    my ($start_handle, $end_handle) = @_;
    my $uuid = pack("S<", 0x2803); # 16-bit UUID for Characteristic Declaration
    return pack("CS<S<a*", 0x08, $start_handle, $end_handle, $uuid);
}

# GATT Enable Notification (ATT Write Request to CCCD)
sub gatt_enable_notify {
    my ($cccd_handle) = @_;
    return pack("CS<S<", 0x12, $cccd_handle, 1);
}

# GATT Descriptor Discovery (ATT Find Information Request)
sub gatt_desc_discovery {
    my ($start_handle, $end_handle) = @_;
    return pack("CS<S<", 0x04, $start_handle, $end_handle);
}

# GATT MTU Request (ATT Exchange MTU Request)
sub gatt_mtu_request {
    my ($mtu) = @_;
    $mtu //= 23; # default MTU size
    return pack("CS<", 0x02, $mtu);
}

# GATT Write Request (ATT Write Request)
sub gatt_write {
    my ($handle, $value) = @_;
    return pack("CS<a*", 0x12, $handle, $value);
}

# GATT Read Request (ATT Read Request)
sub gatt_read {
    my ($handle) = @_;
    return pack("CS<", 0x0A, $handle);
}

# GATT Read By Type Request (for reading characteristics by UUID)
sub gatt_read_by_type {
    my ($start_handle, $end_handle, $uuid) = @_;
    $start_handle //= 0x0001;
    $end_handle   //= 0xFFFF;
    return pack("CS<S<a*", 0x08, $start_handle, $end_handle, pack("S<", $uuid));
}

# GATT Handle Value Indication (ATT Handle Value Indication)
sub gatt_indication {
    my ($handle, $value) = @_;
    return pack("CS<a*", 0x1D, $handle, $value);
}

# GATT Handle Value Confirmation (ATT Handle Value Confirmation)
sub gatt_confirmation {
    return pack("C", 0x1E);
}

# SMP Pairing Request
sub smp_pairing_request {
    my ($io_capability, $oob_flag, $auth_req, $max_enc_key_size, $init_key_dist, $resp_key_dist) = @_;
    $io_capability //= BT_IO_CAP_NO_INPUT_OUTPUT;
    $oob_flag //= 0;  # OOB data not present
    $auth_req //= 0x01;  # Bonding, no MITM
    $max_enc_key_size //= 16;  # Maximum encryption key size
    $init_key_dist //= 0x07;  # LTK, EDIV, Rand, IRK, CSRK
    $resp_key_dist //= 0x07;  # LTK, EDIV, Rand, IRK, CSRK

    return pack("CCCCCCCC", 0x01, $io_capability, $oob_flag, $auth_req,
                $max_enc_key_size, $init_key_dist, $resp_key_dist);
}

# SMP Pairing Response
sub smp_pairing_response {
    my ($io_capability, $oob_flag, $auth_req, $max_enc_key_size, $init_key_dist, $resp_key_dist) = @_;
    $io_capability //= BT_IO_CAP_NO_INPUT_OUTPUT;
    $oob_flag //= 0;
    $auth_req //= 0x01;
    $max_enc_key_size //= 16;
    $init_key_dist //= 0x07;
    $resp_key_dist //= 0x07;

    return pack("CCCCCCCC", 0x02, $io_capability, $oob_flag, $auth_req,
                $max_enc_key_size, $init_key_dist, $resp_key_dist);
}

# SMP Pairing Confirm
sub smp_pairing_confirm {
    my ($confirm_value) = @_;
    $confirm_value //= "\x00" x 16;  # 16-byte confirm value
    return pack("Ca16", 0x03, $confirm_value);
}

# SMP Pairing Random
sub smp_pairing_random {
    my ($random_value) = @_;
    $random_value //= "\x00" x 16;  # 16-byte random value
    return pack("Ca16", 0x04, $random_value);
}

# SMP Security Request
sub smp_security_request {
    my ($auth_req) = @_;
    $auth_req //= 0x01;  # Bonding, no MITM
    return pack("CC", 0x0B, $auth_req);
}

sub cleanup {
    my ($self) = @_;
    close($self->{_socket}) if defined $self->{_socket};
    delete $self->{_socket};
    delete $self->{_fd};
    substr($self->{_outbuffer} //="", 0, length($self->{_outbuffer}//"")) = "";
    substr($self->{_outboxbuffer} //="", 0, length($self->{_outboxbuffer}//"")) = "";
    return;
}

sub pack_sockaddr_bt {
    my ($bt_addr, $l2cap_port, $v1, $v2) = @_;
    return pack "SSa6SS", AF_BLUETOOTH, $l2cap_port, $bt_addr, $v1//L2CAP_CID_ATT, $v2//BDADDR_LE_PUBLIC;
}

sub bt_aton {
    return scalar reverse pack("H12", ($_[0]//BDADDR_ANY) =~ s/://gr);
}

sub pack_sockaddr_bt_rfcomm {
    my ($bt_addr, $v1) = @_;
    return pack "Sa6S", AF_BLUETOOTH, $bt_addr, $v1;
}

sub need_write {
    my ($self) = @_;
    return 1 if exists $self->{_nok};

    # GATT state change/check/handle

    # State machine for GATT discovery and usage
    $self->{_gatt_state} //= 'mtu';

    # Check if security upgrade is needed
    if($self->{_gatt_state} eq 'security_upgrade') {
        my $security_level = $self->{cfg}{l}{security_level} // $::APP_OPTS->{_security_level} // BT_SECURITY_LOW;
        if ($security_level > BT_SECURITY_LOW) {
            logger::info("Requesting security upgrade to level: $security_level");
            $self->{_gatt_state} = 'security_upgrade_sent';
            # Send SMP Security Request
            my $auth_req = ($security_level >= BT_SECURITY_MEDIUM) ? 0x05 : 0x01;  # MITM if medium+
            $self->{_outbuffer} .= smp_security_request($auth_req);
            return 1;
        } else {
            $self->{_gatt_state} = 'mtu';  # Skip security upgrade
        }
    }

    # If we are in 'ready' state, check if we can send data from outbox
    if($self->{_gatt_state} eq 'ready' and $self->{_rx_handle}){
        $self->handle_outbox();
    } elsif($self->{_gatt_state} eq 'mtu') {
        $self->{_gatt_state} = 'mtu_sent';
        # Request the ATT MTU size from the server
        logger::info("Requesting ATT MTU size from server: 256");
        $self->{_outbuffer} .= gatt_mtu_request(256);
    } elsif($self->{_gatt_state} eq 'desc_discovery') {
        $self->{_gatt_state} = 'desc_discovery_sent';
        # Start descriptor discovery
        logger::info(sprintf "Starting GATT Descriptor Discovery for TX Characteristic (handle=0x%04X)", $self->{_tx_handle});
        $self->{_outbuffer} .= gatt_desc_discovery($self->{_tx_handle}+1, 0xFFFF);
    } elsif($self->{_gatt_state} eq 'notify_tx' and defined $self->{_nus_cccd}) {
        # Enable notifications
        logger::info(sprintf "Enabling notifications for TX Characteristic (handle=0x%04X)", $self->{_nus_cccd});
        $self->{_gatt_state} = 'notify_tx_sent';
        $self->{_outbuffer} .= gatt_enable_notify($self->{_nus_cccd});
    } elsif($self->{_gatt_state} eq 'char'){
        # If we have the service handles, start discovery of characteristics
        logger::info(sprintf "Starting GATT Characteristic Discovery for service (start=0x%04X, end=0x%04X)", $self->{_char_start_handle}, $self->{_char_end_handle});
        $self->{_gatt_state} = 'char_discovery_sent';
        $self->{_outbuffer} .= gatt_char_discovery($self->{_char_start_handle}, $self->{_char_end_handle});
    } elsif($self->{_gatt_state} eq 'want_service_discovery') {
        logger::info(sprintf "Sending GATT discovery request for primary services, (start=0x%04X, end=0x%04X)", $self->{_service_start_handle}//1, $self->{_service_end_handle}//0xFFFF);
        $self->{_gatt_state} = 'service_discovery_sent';
        $self->{_outbuffer} .= gatt_discovery_primary($self->{_service_start_handle}, $self->{_service_end_handle});
    } else {
        # If we are not in a state where we can write, return 0
        logger::debug("No data to write, current GATT state: ", $self->{_gatt_state}, ", outbuffer length: ", length($self->{_outbuffer}//""));
    }

    logger::debug("Current outbuffer length: ", length($self->{_outbuffer}//""));
    return 1 if length($self->{_outbuffer}//"");
    return 0;
}

sub need_timeout {
    my ($self) = @_;
    return;
}

sub do_read {
    my ($self, $response) = @_;
    $response //= \(my $_d = '');
    my $r_in_data = "";
    my $r_sz = 512;
    while($::DATA_LOOP){
        my $r = sysread($self->{_socket}, $r_in_data, $r_sz);
        if(defined $r){
            # EOF?
            return 0 if $r == 0;
            local $!;
            my $r_ble_data = $self->handle_ble_response_data($r_in_data);
            $$response .= $r_ble_data if defined $r_ble_data and length($r_ble_data);
            $r_in_data = "";
        } else {
            return 1 if $! == e::EINTR or $! == e::EAGAIN;
            die "problem reading data $self->{_log_info}: $!\n" if $!;
        }
    }
    return 1;
}

sub do_write {
    my ($self) = @_;
    delete $self->{_nok};
    my $n = length($self->{_outbuffer});
    logger::debug(">>WRITE>>", $n, ">>", utils::tohex($self->{_outbuffer}));
    my $w = syswrite($self->{_socket}, $self->{_outbuffer}, $n, 0);
    if(defined $w){
        if($n == $w){
            substr($self->{_outbuffer}, 0, $n, '');
        } else {
            substr($self->{_outbuffer}, 0, $w, '');
        }
    } else {
        return if $! == e::EINTR or $! == e::EAGAIN;
        die "problem writing data $self->{_log_info}: $!\n" if $!;
    }
    return;
}

# Helper function to format 128-bit UUID with dashes like gatttool
sub format_128bit_uuid {
    my ($uuid_raw) = @_;
    # little-endian in BLE, so reverse the bytes
    $uuid_raw = reverse $uuid_raw;
    my $uuid_hex = uc(unpack('H*', $uuid_raw));
    # Input: 4 character hex string (16-bit UUID)
    if (length($uuid_hex) == 4) {
        # 16-bit UUID - display in gatttool format
        $uuid_hex = "0000${uuid_hex}00001000800000805F9B34FB"; # convert 16-bit to 128-bit UUID format
    }
    # Input: 32 character hex string (no dashes)
    # Output: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx format
    return lc $uuid_hex unless length($uuid_hex) == 32;
    return uc join('-',substr($uuid_hex,0,8),substr($uuid_hex,8,4),substr($uuid_hex,12,4),substr($uuid_hex,16,4),substr($uuid_hex,20,12));
}

# pick proper defaults, even if not ble::uart (NUS)
sub RX_HANDLE_UUID {"6E400002-B5A3-F393-E0A9-E50E24DCCA9E"} # RX Characteristic UUID
sub TX_HANDLE_UUID {"6E400003-B5A3-F393-E0A9-E50E24DCCA9E"} # TX Characteristic UUID
sub SERVICE_UUID   {"6E400001-B5A3-F393-E0A9-E50E24DCCA9E"} # NUS Service UUID

sub char_uuid {
    my ($self, $handle, $val_handle, $uuid, $props) = @_;

    $uuid = uc($uuid);

    # Compare against characteristic UUIDs (normalize both to uppercase, no dashes)
    if ($uuid eq $self->RX_HANDLE_UUID()) {
        $self->{_rx_handle} = $val_handle;
    } elsif ($uuid eq $self->TX_HANDLE_UUID()) {
        $self->{_tx_handle} = $val_handle;
    } else {
        logger::info(sprintf "Found characteristic: handle=0x%04X val_handle=0x%04X uuid=%s props=0x%02X (not RX/TX)", $handle, $val_handle, $uuid, $props);
    }
    if(defined $self->{_rx_handle} and defined $self->{_tx_handle}) {
        logger::info(sprintf "Found characteristic: RX=0x%04X TX=0x%04X", $self->{_rx_handle}, $self->{_tx_handle});
        $self->{_gatt_state} = 'desc_discovery';
        return 1;
    }
    return 0;
}

sub service_uuid {
    my ($self, $start, $end, $uuid) = @_;
    # Compare against UUID (normalize both to uppercase, no dashes) - only for automatic discovery
    my $is_primary_discovery = $self->{_primary_discovery_active};
    if (!$is_primary_discovery && uc($uuid) eq $self->SERVICE_UUID()) {
        logger::info(sprintf "Found service: start=0x%04X end=0x%04X uuid=%s", $start, $end, $uuid);
        $self->{_gatt_state}        = 'char';
        $self->{_char_start_handle} = $start;
        $self->{_char_end_handle}   = $end;
        return 1;
    }
    return 0;
}

sub handle_outbox {
    my ($self) = @_;
    logger::debug("Handling outbox, current outboxbuffer length: ", length($self->{_outboxbuffer}//""));
    return;
}

sub handle_ble_response_data {
    my ($self, $data) = @_;
    return unless defined $data && length $data;

    my $opcode = unpack('C', $data);
    logger::debug(sprintf "<<GATT<< opcode=0x%02X (%s) data=[%s]", $opcode, get_att_opcode_name($opcode), utils::tohex($data));

    # Check if we should handle this as SMP (Security Manager Protocol) instead of ATT
    # SMP messages have opcodes 0x01-0x0B and are used during security/pairing operations
    if ($opcode >= 0x01 && $opcode <= 0x0B && length($data) >= 2 &&
        ($self->{_gatt_state} =~ /^security/ ||
         ($self->{cfg}{l}{security_level} // $::APP_OPTS->{_security_level} // BT_SECURITY_LOW) > BT_SECURITY_LOW)) {

        # SMP (Security Manager Protocol) messages
        logger::debug(sprintf "<<SMP<< opcode=0x%02X", $opcode);
        my $smp_opcode_handler = lc sprintf("_smp_opcode_0x%02X", $opcode);
        if(UNIVERSAL::can($self, $smp_opcode_handler)){
            logger::debug("Handling SMP opcode 0x$opcode with custom handler $smp_opcode_handler");
            $self->$smp_opcode_handler($data);
        } else {
            logger::info(sprintf "Unhandled SMP opcode: 0x%02X", $opcode);
        }
        return;
    }

    # Handle as ATT (Attribute Protocol) message
    my $att_opcode_handler = lc sprintf("_att_opcode_0x%02X", $opcode);
    if(UNIVERSAL::can($self, $att_opcode_handler)){
        logger::debug("Handling opcode 0x$opcode with custom handler $att_opcode_handler");
        return $self->$att_opcode_handler($data);
    } else {
        logger::info(sprintf "Unhandled GATT/ATT opcode: 0x%02X (%s)", $opcode, get_att_opcode_name($opcode));
        return;
    }
    return;
}

# SMP Pairing Request
sub _smp_opcode_0x01 {
    my ($self, $data) = @_;
    # Format: opcode(1) | io_cap(1) | oob(1) | auth_req(1) | max_key_size(1) | init_key_dist(1) | resp_key_dist(1)
    logger::info("Received SMP Pairing Request");
    my ($io_cap, $oob, $auth_req, $max_key_size, $init_key_dist, $resp_key_dist) = unpack('xCCCCCC', $data);
    logger::debug(sprintf "Pairing Request: io_cap=0x%02X oob=0x%02X auth_req=0x%02X", $io_cap, $oob, $auth_req);

    # Send Pairing Response
    my $our_io_cap = $self->{cfg}{l}{io_capability} // $::APP_OPTS->{_io_capability} // BT_IO_CAP_NO_INPUT_OUTPUT;
    my $our_auth_req = ($self->{cfg}{l}{security_level} // $::APP_OPTS->{_security_level} // BT_SECURITY_LOW) >= BT_SECURITY_MEDIUM ? 0x05 : 0x01;

    $self->{_outbuffer} .= smp_pairing_response($our_io_cap, 0, $our_auth_req, 16, 0x07, 0x07);
    logger::info("Sent SMP Pairing Response");
    return;
}

# SMP Pairing Response
sub _smp_opcode_0x02 {
    my ($self, $data) = @_;
    # Format: opcode(1) | io_capability(1) | oob_flag(1) | auth_req(1) | max_enc_key_size(1) | init_key_dist(1) | resp_key_dist(1)
    logger::info("Received SMP Pairing Response");
    my ($io_cap, $oob, $auth_req, $max_key_size, $init_key_dist, $resp_key_dist) = unpack('xCCCCCC', $data);
    logger::debug(sprintf "Pairing Response: io_cap=0x%02X oob=0x%02X auth_req=0x%02X", $io_cap, $oob, $auth_req);
    return;
}

# SMP Pairing Confirm
sub _smp_opcode_0x03 {
    my ($self, $data) = @_;
    # Format: opcode(1) | confirm_value(16)
    logger::info("Received SMP Pairing Confirm");
    # Send our own confirm (simplified for demo)
    $self->{_outbuffer} .= smp_pairing_confirm();
    return;
}

# SMP Pairing Random
sub _smp_opcode_0x04 {
    my ($self, $data) = @_;
    # Format: opcode(1) | random(16)
    logger::info("Received SMP Pairing Random");
    # Send our own random (simplified for demo)
    $self->{_outbuffer} .= smp_pairing_random();
    return;
}

# SMP Pairing Failed
sub _smp_opcode_0x05 {
    my ($self, $data) = @_;
    # Format: opcode(1) | reason(1)
    my ($reason) = unpack('xC', $data);
    logger::error(sprintf "SMP Pairing Failed: reason=0x%02X", $reason);
    return;
}

# SMP Security Request
sub _smp_opcode_0x0b {
    my ($self, $data) = @_;
    # Format: opcode(1) | auth_req(1)
    logger::info("Received SMP Security Request");
    my ($auth_req) = unpack('xC', $data);
    logger::debug(sprintf "Security Request: auth_req=0x%02X", $auth_req);

    # Initiate pairing if we have higher security requirements
    my $our_security = $self->{cfg}{l}{security_level} // $::APP_OPTS->{_security_level} // BT_SECURITY_LOW;
    if ($our_security > BT_SECURITY_LOW) {
        my $our_io_cap = $self->{cfg}{l}{io_capability} // $::APP_OPTS->{_io_capability} // BT_IO_CAP_NO_INPUT_OUTPUT;
        my $our_auth_req = ($our_security >= BT_SECURITY_MEDIUM) ? 0x05 : 0x01;
        $self->{_outbuffer} .= smp_pairing_request($our_io_cap, 0, $our_auth_req, 16, 0x07, 0x07);
        logger::info("Initiated SMP Pairing Request");
    }
    return;
}

# Error Response
sub _att_opcode_0x01 {
    my ($self, $data) = @_;
    # Format: opcode(1) | req_opcode(1) | handle(2) | error_code(1)
    my ($req_opcode, $handle, $err_code) = unpack('xCS<C', $data);
    # map the error code to a human-readable message
    # this is not exhaustive, but covers common cases
    my $emsg = $err_msg{$err_code};
    if(!defined $emsg){
        if ($err_code >= 0x15 and $err_code <= 0x9F) {
            $emsg = sprintf("Reserved Error Code: 0x%02X", $err_code);
        } elsif ($err_code >= 0xE0 and $err_code <= 0xFF) {
            $emsg = sprintf("Vendor Specific Error Code: 0x%02X", $err_code);
        } else {
            $emsg = sprintf("Unknown Error Code: 0x%02X", $err_code);
        }
    }

    # Check if this is an expected "Attribute Not Found" error at the end of manual discovery
    my $is_manual_discovery_end = 0;
    if ($err_code == 0x0A) { # Attribute Not Found
        if (($req_opcode == 0x10 && $self->{_primary_discovery_active}) ||     # Primary service discovery
            ($req_opcode == 0x04 && $self->{_char_desc_discovery_active})) {   # Characteristic descriptor discovery
            $is_manual_discovery_end = 1;
        }
    }

    # Only log as error if it's not an expected end-of-discovery error
    unless ($is_manual_discovery_end) {
        logger::error(sprintf "ATT Error Response: req_opcode=0x%02X handle=0x%04X code=0x%02X msg=%s", $req_opcode, $handle, $err_code, $emsg);
    }
    return;
}

# Exchange MTU Request
sub _att_opcode_0x02 {
    my ($self, $data) = @_;

    # Client is requesting to negotiate MTU size
    my ($client_mtu) = unpack('xS<', $data);
    logger::info(sprintf "Exchange MTU Request: client_mtu=%d", $client_mtu);

    # Respond with our MTU (we can use the same as client or our preferred size)
    my $our_mtu = $client_mtu; # Just echo back their MTU
    my $mtu_response = pack('CS<', 0x03, $our_mtu);
    $self->{_outbuffer} .= $mtu_response;
    logger::debug(sprintf "Sent Exchange MTU Response: our_mtu=%d", $our_mtu);
    return;
}

# ATT Server receive MTU size
sub _att_opcode_0x03 {
    my ($self, $data) = @_;
    my ($mtu) = unpack('xS<', $data);
    if (!defined $mtu || $mtu < 23 || $mtu > 517) {
        logger::warn("Invalid MTU size received from server, using default of 23 bytes");
        return;
    }
    logger::info(sprintf "ATT Server MTU size: %d bytes", $mtu);
    $self->{_att_mtu} = $mtu;
    # Set the initial state to 'want_service_discovery' to start service discovery
    $self->{_gatt_state}           = 'want_service_discovery';
    $self->{_service_start_handle} = 0x0001;
    $self->{_service_end_handle}   = 0xFFFF;
    return;
}

# Find Information Request
sub _att_opcode_0x04 {
    my ($self, $data) = @_;

    # Client is requesting descriptor information
    my ($start_handle, $end_handle) = unpack('xS<S<', $data);
    logger::info(sprintf "Find Information Request: start=0x%04X end=0x%04X", $start_handle, $end_handle);

    # Respond with "Attribute Not Found"
    my $error_response = pack('CCS<C', 0x01, 0x04, $start_handle, 0x0A);
    $self->{_outbuffer} .= $error_response;
    logger::debug("Sent ATT Error Response for Find Information Request: Attribute Not Found");
    return;
}

# Find Information Response (Descriptor Discovery)
sub _att_opcode_0x05 {
    my ($self, $data) = @_;
    my ($fmt) = unpack('xC', $data); # 0x01 = 16-bit UUID, 0x02 = 128-bit UUID
    my $entry_len = $fmt == 1 ? 4 : 18;
    my $count = (length($data) - 2) / $entry_len;

    # Check if this is a manual char-desc discovery request
    my $is_char_desc_discovery = $self->{_char_desc_discovery_active};
    my $last_handle = 0;

    for (my $i = 0; $i < $count; $i++) {
        my $entry = substr($data, 2 + $i * $entry_len, $entry_len);
        my ($handle, $uuid_raw);
        if ($fmt == 1) {
            ($handle, $uuid_raw) = unpack('S<S<', $entry);
            $uuid_raw = pack('S<', $uuid_raw);
        } else {
            ($handle, $uuid_raw) = unpack('S<a16', $entry);
        }
        $last_handle = $handle if $handle > $last_handle;

        my $uuid = format_128bit_uuid($uuid_raw);
        if ($is_char_desc_discovery) {
            # Display in gatttool format for manual discovery
            logger::info(sprintf "handle: 0x%04x, uuid: %s\n", $handle, $uuid);
        } else {
            # Normal discovery logging for automatic discovery
            logger::info(sprintf "Descriptor: handle=0x%04X uuid=0x%s", $handle, $uuid);
        }

        # CCCD UUID: 00002902-0000-1000-8000-00805f9b34fb, or 2902 in 16-bit format
        if (lc($uuid) eq "2902" || lc($uuid) eq "00002902-0000-1000-8000-00805f9b34fb") {
            $self->{_nus_cccd} = $handle;
            logger::info(sprintf "Found CCCD for TX Characteristic at handle=0x%04X", $handle);
            $self->{_gatt_state} = 'notify_tx' unless $is_char_desc_discovery;
            return unless $is_char_desc_discovery;
        }
    }

    # Handle continuation of discovery for manual requests
    if ($is_char_desc_discovery) {
        my $discovery_end = $self->{_char_desc_discovery_end} // 0xFFFF;
        if ($last_handle && $last_handle < $discovery_end) {
            # Continue discovery
            my $discovery_pdu = ble::gatt_desc_discovery($last_handle + 1, $discovery_end);
            $self->{_outbuffer} .= $discovery_pdu;
        } else {
            # Discovery completed
            $self->{_char_desc_discovery_active} = 0;
            logger::lsprintf("Characteristic descriptor discovery completed.\n");
        }
    }
    return;
}

# Read By Type Request
sub _att_opcode_0x08 {
    my ($self, $data) = @_;

    # Client is requesting characteristics or other attributes by type
    my ($start_handle, $end_handle, $uuid_raw) = unpack('xS<S<a*', $data);
    my $uuid_hex = format_128bit_uuid($uuid_raw);
    logger::info(sprintf "Read By Type Request: start=0x%04X end=0x%04X uuid=%s", $start_handle, $end_handle, $uuid_hex);

    # Respond with "Attribute Not Found" since we're not a GATT server
    my $error_response = pack('CCS<C', 0x01, 0x08, $start_handle, 0x0A);
    $self->{_outbuffer} .= $error_response;
    logger::debug("Sent ATT Error Response for Read By Type Request: Attribute Not Found");
    return;
}

# Read By Type Response (Characteristic Discovery)
sub _att_opcode_0x09 {
    my ($self, $data) = @_;

    # Check if this is a response to a Device Name info request
    return &{$self->{_info_request_active}}($self, $data) if defined $self->{_info_request_active};

    my ($len) = unpack('xC', $data);
    if (!defined $len || $len < 7 || $len > 21) {
        logger::error("Invalid characteristic entry length in Read By Type Response: $len");
        return;
    }
    my $count = (length($data) - 2) / $len;
    my $last_val_handle = 0;
    for (my $i = 0; $i < $count; $i++) {
        # Format: opcode(1) length(1) handle(2) properties(1) value_handle(2) uuid(2/16)
        my $entry = substr($data, 2 + $i * $len, $len);
        # Format: handle(2) properties(1) value_handle(2) uuid(2/16)
        my ($handle, $props, $val_handle, $uuid_raw) = unpack('S<CS<a*', $entry);
        $last_val_handle = $val_handle if $val_handle > $last_val_handle;

        # Handle UUIDs
        my $uuid = format_128bit_uuid($uuid_raw);
        logger::info(sprintf "  Char: handle=0x%04X val_handle=0x%04X uuid=%s", $handle, $val_handle, $uuid);

        # handle characteristic UUIDs
        return if $self->char_uuid($handle, $val_handle, $uuid, $props);
    }

    # Continue discovery if not all characteristics are retrieved
    if (defined $self->{_char_end_handle} && $last_val_handle && $last_val_handle < $self->{_char_end_handle}) {
        $self->{_gatt_state}        = 'char';
        $self->{_char_start_handle} = $last_val_handle + 1;
        $self->{_char_end_handle}   = $self->{_char_end_handle} // 0xFFFF;
    }
    return;
}

# Read Request
sub _att_opcode_0x0a {
    my ($self, $data) = @_;
    # Client is requesting to read a specific attribute
    my ($handle) = unpack('xS<', $data);
    logger::info(sprintf "Read Request: handle=0x%04X", $handle);

    # Respond with "Attribute Not Found"
    my $error_response = pack('CCS<C', 0x01, 0x0A, $handle, 0x0A);
    $self->{_outbuffer} .= $error_response;
    logger::debug("Sent ATT Error Response for Read Request: Attribute Not Found");
    return;
}

# Read Blob Request
sub _att_opcode_0x0c {
    my ($self, $data) = @_;
    # Client is requesting to read a long attribute
    my ($handle, $offset) = unpack('xS<S<', $data);
    logger::info(sprintf "Read Blob Request: handle=0x%04X offset=%d", $handle, $offset);

    # Respond with "Attribute Not Found"
    my $error_response = pack('CCS<C', 0x01, 0x0C, $handle, 0x0A);
    $self->{_outbuffer} .= $error_response;
    logger::debug("Sent ATT Error Response for Read Blob Request: Attribute Not Found");
    return;
}

# Read Multiple Request
sub _att_opcode_0x0e {
    my ($self, $data) = @_;
    # Client is requesting to read multiple attributes at once
    my $handles_data = substr($data, 1);
    my @handles;
    for (my $i = 0; $i < length($handles_data); $i += 2) {
        push @handles, unpack('S<', substr($handles_data, $i, 2));
    }
    logger::info(sprintf "Read Multiple Request: handles=[%s]", join(', ', map { sprintf("0x%04X", $_) } @handles));

    # Respond with "Attribute Not Found" for the first handle
    my $error_response = pack('CCS<C', 0x01, 0x0E, $handles[0] // 0x0001, 0x0A);
    $self->{_outbuffer} .= $error_response;
    logger::debug("Sent ATT Error Response for Read Multiple Request: Attribute Not Found");
    return;
}

# Read By Group Type Request
sub _att_opcode_0x10 {
    my ($self, $data) = @_;
    # The remote device is treating us as a GATT server and requesting our services
    # We should respond appropriately - typically with an error indicating we don't have services
    my ($start_handle, $end_handle, $uuid_raw) = unpack('xS<S<a*', $data);
    my $uuid = format_128bit_uuid($uuid_raw);
    logger::info(sprintf "Read By Group Type Request: start=0x%04X end=0x%04X uuid=%s", $start_handle, $end_handle, $uuid);

    # We're acting as a GATT client, not server, so respond with "Attribute Not Found"
    # ATT Error Response: opcode(0x01) | req_opcode(0x10) | handle(2) | error_code(0x0A)
    my $error_response = pack('CCS<C', 0x01, 0x10, $start_handle, 0x0A);
    $self->{_outbuffer} .= $error_response;
    logger::debug("Sent ATT Error Response: Attribute Not Found (we are not a GATT server)");
    return;
}

# Read By Group Type Response (Service Discovery)
sub _att_opcode_0x11 {
    my ($self, $data) = @_;
    # Format: opcode(1) | length(1) | [handle(2) end_handle(2) uuid(2/16)]*
    my ($len) = unpack('xC', $data);
    if( !defined $len || $len < 6 || $len > 20) {
        logger::error("Invalid service entry length in Read By Group Type Response: $len");
        return;
    }
    my $count = (length($data) - 2) / $len;
    logger::info(sprintf "Service Discovery Response: %d services, entry len=%d", $count, $len);

    # Check if this is a manual primary service discovery request
    my $is_primary_discovery = $self->{_primary_discovery_active};

    my $last_end = 0;
    for (my $i = 0; $i < $count; $i++) {
        my $entry = substr($data, 2 + $i * $len, $len);
        my ($start, $end, $uuid_raw) = unpack('S<S<a*', $entry);
        $last_end = $end if $end > $last_end;

        # Handle UUIDs
        my $uuid = format_128bit_uuid($uuid_raw);
        if ($is_primary_discovery) {
            # Display in gatttool format for manual discovery
            logger::info(sprintf " attr handle: 0x%04x, end grp handle: 0x%04x uuid: %s\n", $start, $end, $uuid);
        } else {
            # Normal discovery logging for automatic discovery
            logger::info(sprintf "  Service: start=0x%04X end=0x%04X uuid=%s", $start, $end, $uuid);
        }

        # handle service UUIDs
        return if !$is_primary_discovery and $self->service_uuid($start, $end, $uuid);
    }

    # Handle continuation of discovery
    if ($is_primary_discovery) {
        # For manual primary discovery, check if we should continue
        my $discovery_end = $self->{_primary_discovery_end} // 0xFFFF;
        if ($last_end && $last_end < $discovery_end) {
            # Continue discovery
            $self->{_outbuffer} .= ble::gatt_discovery_primary($last_end + 1, $discovery_end);
        } else {
            # Discovery completed
            $self->{_primary_discovery_active} = 0;
            logger::info("Primary service discovery completed");
        }
    } else {
        # Normal automatic discovery continuation
        if ($last_end && $last_end < 0xFFFF) {
            $self->{_gatt_state}           = 'want_service_discovery';
            $self->{_service_start_handle} = $last_end + 1;
            $self->{_service_end_handle}   = 0xFFFF; # Continue until end
        }
    }
    return;
}

# Write Request
sub _att_opcode_0x12 {
    my ($self, $data) = @_;
    # Client is requesting to write to an attribute
    my ($handle) = unpack('xS<', $data);
    my $value = substr($data, 3);
    logger::info(sprintf "Write Request: handle=0x%04X value=[%s]", $handle, utils::tohex($value));

    # Respond with "Attribute Not Found"
    my $error_response = pack('CCS<C', 0x01, 0x12, $handle, 0x0A);
    $self->{_outbuffer} .= $error_response;
    logger::debug("Sent ATT Error Response for Write Request: Attribute Not Found");
    return;
}

# Write Response (for enabling notifications)
sub _att_opcode_0x13 {
    my ($self, $data) = @_;
    # This is a response to our Write Request (e.g., enabling notifications)
    logger::debug("GATT Write Response received, state: ", $self->{_gatt_state});
    if ($self->{_gatt_state} eq 'notify_tx_sent') {
        $self->{_gatt_state} = 'ready';
        logger::debug(sprintf "ready: RX=0x%04X TX=0x%04X", $self->{_rx_handle}//0, $self->{_tx_handle}//0);
    }
    return;
}

# Prepare Write Request
sub _att_opcode_0x16 {
    my ($self, $data) = @_;
    # Client is starting a long write operation
    my ($handle, $offset) = unpack('xS<S<', $data);
    my $value = substr($data, 5);
    logger::info(sprintf "Prepare Write Request: handle=0x%04X offset=%d value=[%s]",
                 $handle, $offset, utils::tohex($value));

    # Respond with "Attribute Not Found"
    my $error_response = pack('CCS<C', 0x01, 0x16, $handle, 0x0A);
    $self->{_outbuffer} .= $error_response;
    logger::debug("Sent ATT Error Response for Prepare Write Request: Attribute Not Found");
    return;
}

# Execute Write Request
sub _att_opcode_0x18 {
    my ($self, $data) = @_;
    # Client is executing prepared writes
    my ($flags) = unpack('xC', $data);
    logger::info(sprintf "Execute Write Request: flags=0x%02X", $flags);

    # Respond with "Attribute Not Found" - using handle 0x0001 as default
    my $error_response = pack('CCS<C', 0x01, 0x18, 0x0001, 0x0A);
    $self->{_outbuffer} .= $error_response;
    logger::debug("Sent ATT Error Response for Execute Write Request: Attribute Not Found");
    return;
}

# Handle Value Notification
sub _att_opcode_0x1b {
    my ($self, $data) = @_;
    # Server is sending a notification (no confirmation needed)
    my ($handle) = unpack('xS<', $data);
    my $value = substr($data, 3);
    if (($handle//0) == ($self->{_tx_handle}//0)) {
        logger::debug("RX Notification: ", length($value), " data: ", utils::tohex($value));
        return $value if length($value) > 0;
    }
    return;
}

# Handle Value Indication
sub _att_opcode_0x1d {
    my ($self, $data) = @_;
    my ($handle) = unpack('xS<', $data);
    my $value = substr($data, 3);
    logger::debug(sprintf "Handle Value Indication: handle=0x%04X value=[%s]", $handle, utils::tohex($value));

    # Handle Value Indication requires a Handle Value Confirmation (0x1E) response
    my $confirmation = pack('C', 0x1E);
    $self->{_outbuffer} .= $confirmation;
    logger::debug("Sent Handle Value Confirmation");

    # If this is from our TX handle, return the data
    if (($handle//0) == ($self->{_tx_handle}//0)) {
        logger::debug("RX Indication: ", length($value), " data: ", utils::tohex($value));
        return $value if length($value) > 0;
    }
    return;
}

# Handle Value Confirmation
sub _att_opcode_0x1e {
    my ($self, $data) = @_;
    # This is a confirmation for an indication we sent
    logger::debug("Received Handle Value Confirmation");
    # No action needed - just acknowledgment that our indication was received
    return;
}

# Write Command (no response expected)
sub _att_opcode_0x52 {
    my ($self, $data) = @_;
    # Client is writing to an attribute without expecting a response
    my ($handle) = unpack('xS<', $data);
    my $value = substr($data, 3);
    logger::info(sprintf "Write Command: handle=0x%04X value=[%s] (no response sent)", $handle, utils::tohex($value));
    # No response needed for Write Command
    return;
}


package ble::uart;

use strict;
BEGIN {@ble::uart::ISA = qw(ble)};

# constants for BLE UART (Nordic UART Service) UUIDs - configurable via environment variables
sub RX_HANDLE_UUID {"6E400002-B5A3-F393-E0A9-E50E24DCCA9E"} # RX Characteristic UUID
sub TX_HANDLE_UUID {"6E400003-B5A3-F393-E0A9-E50E24DCCA9E"} # TX Characteristic UUID
sub SERVICE_UUID   {"6E400001-B5A3-F393-E0A9-E50E24DCCA9E"} # NUS Service UUID

sub handle_outbox {
    my ($self) = @_;
    logger::debug(sprintf "NUS ready, RX handle: 0x%04X, TX handle: 0x%04X", $self->{_rx_handle}//0, $self->{_tx_handle}//0);

    # If we have a RX handle, check if there is data in the outbox buffer
    return unless length($self->{_outboxbuffer}//"");

    # is there a RX handle set?
    my $r = index($self->{_outboxbuffer}, "\n");
    if ($r == -1) {
        logger::debug("No newline in outbox buffer, waiting for more data");
        return;
    }

    my $_out = substr($self->{_outboxbuffer}, 0, $r + 1);
    # massage the buffer so a \n becomes a \r\n
    # this is only needed for AT command mode, note that if \n is already preceded with \r, it will not be changed
    $_out =~ s/\r?\n$/\r\n/ if $self->{cfg}{l}{uart_at} // 1;
    logger::debug(">>OUTBOX>>", $_out, ">>", length($_out), " bytes to write to NUS (after massage): ", utils::tohex($_out));

    if(length($_out) > $self->{_att_mtu}){
        logger::error("Data to write to NUS is too long: ", length($_out), " bytes, max is ", $self->{_att_mtu}, " bytes");
    } else {
        logger::debug("Data to write to NUS is within MTU limits: ", length($_out), " bytes");

        # append to the outbuffer
        # this is the buffer that will be written to the socket
        # it is not written immediately, but only when the socket is ready
        # to write
        my $ble_data = ble::gatt_write($self->{_rx_handle}, $_out);
        if(defined $ble_data and length($ble_data) > 0){
            substr($self->{_outboxbuffer}, 0, $r + 1, '');
            logger::debug(">>BLE DATA>>", length($ble_data), " bytes to write to NUS");
            $self->{_outbuffer} .= $ble_data;
        } else {
            logger::error("Problem packing data for NUS write");
        }
    }
    return;
}


package utils;

use strict;

sub load_cpan {
    my ($c) = @_;
    eval "require $c";
    return $c unless $@;
    return;
}

sub usage {
    my (%msg) = @_;
    no warnings 'once';
    utils::load_cpan("Pod::Usage");
    local $ENV{MANPAGER} = $ENV{MANPAGER}||$::ENV{MANPAGER}||"/usr/bin/less";
    local $ENV{PAGER}    = $ENV{PAGER}||$ENV{MANPAGER};
    my $p_fn = do {
        local $0 = $::DOLLAR_ZERO // $0;
        utils::load_cpan("FindBin");
        FindBin::again();
        "$FindBin::Bin/$FindBin::Script";
    };
    open(my $mfh, '>>', \(my $_d));
    Pod::Usage::pod2usage(
        -input    => $p_fn,
        -sections => "NAME|SYNOPSIS|ARGUMENTS",
        -output   => $mfh,
        %msg,
        -exitval  => 'NOEXIT',
        -verbose  => 99,
    );
    logger::lsprintf("\n$_d");
    exit $msg{-exitval} if defined $msg{-exitval} && $msg{-exitval} ne 'NOEXIT';
}

sub manpage {
    my ($do_exit) = @_;
    # if both "pod2man" and "man" exists, use those, else, fallback to usage()
    my $ok_man = 1;
    system("which pod2man >/dev/null 2>&1 </dev/null") == 0 and
    system("which man     >/dev/null 2>&1 </dev/null") == 0 or do {$ok_man = 0};
    if($ok_man){
        local $ENV{MANPAGER} = $ENV{MANPAGER}||$::ENV{MANPAGER}||"less";
        my $p_fn = do {
            local $0 = $::DOLLAR_ZERO // $0;
            utils::load_cpan("FindBin");
            FindBin::again();
            "$FindBin::Bin/$FindBin::Script";
        };
        system("pod2man $p_fn|man /dev/stdin");
        exit 0 if $do_exit;
    } else {
        utils::usage(-verbose => 2, -exitval => 1, -output => undef);
    }
    return;
}

sub tohex {
    my ($data) = @_;
    return join('', map {sprintf '%02X', ord($_)} split //, $data);
}


package logger;

use strict;
no warnings 'once';

our $_json_printer;

BEGIN {
*log_fatal = *fatal;
*log_error = *error;
*log_info  = *info;
*log_debug = *debug;
*_pr_msg   = *_def_pr_msg;
};

sub _def_pr_msg {
    no warnings;
    print STDERR @_;
}

sub log_printer {
    my ($r_sub) = @_;
    no strict 'refs';
    no warnings 'redefine';
    *_pr_msg = $r_sub if defined $r_sub and ref($r_sub) eq "CODE";
    return;
}

sub _loglevel {
    my ($rx) = @_;
    # do a caller() to avoid recursion
    my $i = 1;
    while(my @c = caller($i)){
        return if $c[3] and $c[3] eq "logger::do_log";
        $i++;
    }
    # default to error|info|debug if not provided
    $rx //= qr/^(error|info|debug)$/;
    return lc(utils::cfg("loglevel", "info")) =~ $rx || utils::cfg("DEBUG", 0);
}

sub fatal {
    my (@msg) = @_;
    return unless _loglevel(qr/^(fatal|error|info|debug)$/);
    my $msg = do_log("error", @msg);
    die "$msg\n";
}

sub error {
    my (@msg) = @_;
    return unless _loglevel(qr/^(error|info|debug)$/);
    return do_log("error", @msg);
}

sub lsprintf {
    my ($fmt, @args) = @_;
    _pr_msg(sprintf($fmt, @args));
    return;
}

sub info {
    my (@msg) = @_;
    return unless _loglevel(qr/^(info|debug)$/);
    return do_log("info", @msg);
}

sub debug {
    my (@msg) = @_;
    return unless _loglevel(qr/^(debug)/);
    no warnings 'once';
    my @r_msg = eval {
        return do_log("debug", map {defined $_ and ref($_)
            ?do {
                $_ = eval {
                    utils::load_cpan("JSON");
                    $_json_printer //= JSON->new->canonical->allow_nonref->allow_unknown->allow_blessed->convert_blessed->allow_tags->indent(0);
                    $_json_printer->encode($_);
                };
                $_//""
            }
            :$_} @msg);
    };
    if($@){
        chomp(my $err = $@);
        logger::error("problem logging debug message: $err");
        return;
    }
    return @r_msg;
}

sub do_log {
    my ($w, @msg) = @_;
    local $::LOG_PREFIX = $::LOG_PREFIX // "";
    @msg =
        map {split m/\n/, $_//""}
        join("",
        map {defined $_ and ref($_)
            ?do {
                $_ = eval {
                    utils::load_cpan("JSON");
                    $_json_printer //= JSON->new->canonical->allow_nonref->allow_unknown->allow_blessed->convert_blessed->allow_tags->indent(0);
                    $_json_printer->encode($_);
                };
                $_//"";
            }
            :$_//""
        } @msg);
    my $msg = join("\n", map {sprintf("%02d:%02d:%02d", reverse ((gmtime())[0,1,2]))." [$$] [$w]: $::LOG_PREFIX$_"} map {split m/\n/, $_//""} @msg);
    if($w eq "debug"){
        _def_pr_msg("$msg\n");
    } else {
        _pr_msg("$msg\n");
    }
    return $msg;
}


package utils;

use strict;

sub cfg {
    my ($k, $default_v, $nm, $do_exception, $r) = @_;
    no warnings 'once';
    $nm //= $::APP_MODULE // "";
    my $env_k_m = uc(($::APP_NAME?$::APP_NAME."_":"")."${nm}_$k") =~ s/\W/_/gr;
    my $env_k_a = uc(($::APP_NAME?$::APP_NAME."_":"").$k) =~ s/\W/_/gr;
    my $v = ($r and UNIVERSAL::can($r, "variable") and $r->variable(lc($env_k_a)))
        // $::APP_ENV{$env_k_m}
        // $::APP_ENV{$env_k_a}
        // $::APP_CFG{$env_k_m}
        // $::APP_CFG{$env_k_a}
        // $ENV{$env_k_m}
        // $ENV{$env_k_a}
        // $default_v;
    die "need '$k' config or $env_k_m/$env_k_a ENV variable\n"
        if $do_exception and not defined $v;
    return $v;
}

sub set_cfg {
    my ($k, $v) = @_;
    my $env_k_a = uc(($::APP_NAME?$::APP_NAME."_":"").$k) =~ s/\W/_/gr;
    $::APP_CFG{$env_k_a} = $v;
    return $v;
}


__END__
=pod

=head1 NAME

ble_uart.pl - BLE UART (Nordic UART Service) bridge in Perl

=head1 SYNOPSIS

B<ble_uart.pl> B<[>OPTIONSB<]> [XX:XX:XX:XX:XX:XX[,option=value ...][,...]]

=head1 DESCRIPTION

This script connects to one or more BLE devices implementing the Nordic UART
Service (NUS), discovers the UART RX/TX characteristics, and allows simple
UART-style read/write over BLE. It is intended for use with ESP32/ESP-AT or
similar BLE UART bridges.

Input from the terminal is sent to the BLE device, and data received from the
device is printed to the terminal with a distinct prompt and color. Multiple
connections can be managed interactively.

=head1 NORDIC UART SERVICE (NUS)

This script communicates with BLE devices that implement the Nordic UART Service
(NUS), a standard BLE service for serial-style data transfer over Bluetooth Low
Energy. NUS is commonly used on Nordic Semiconductor devices (such as
nRF52/nRF53 series) and is supported by many ESP32 BLE UART firmware projects,
including ESP-AT.

NUS provides a simple way to send and receive data as if over a UART/serial
port, but using BLE characteristics for RX (write) and TX (notify). This allows
wireless, bidirectional, low-latency communication between a host (like this
script) and a BLE device.

=head2 NUS Service and Characteristics

The Nordic UART Service uses the following UUIDs:

=over 4

=item * Service UUID: C<6E400001-B5A3-F393-E0A9-E50E24DCCA9E>

=item * TX Characteristic (notify, device to client): C<6E400003-B5A3-F393-E0A9-E50E24DCCA9E>

=item * RX Characteristic (write, client to device): C<6E400002-B5A3-F393-E0A9-E50E24DCCA9E>

=back

The host writes data to the RX characteristic, and receives notifications from
the TX characteristic.

For more information, see the official Nordic documentation:

L<https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/libraries/bluetooth/services/nus.html>

=head1 ARGUMENTS

The script expects one or more Bluetooth addresses of BLE devices implementing
the Nordic UART Service (NUS).

The format is:

    XX:XX:XX:XX:XX:XX[,option=value][,...]

Where:

=over 4

=item XX:XX:XX:XX:XX:XX

The Bluetooth address of the device to connect to.

=item option=value

An optional configuration option

=over 6

=item B<uart_at=0|1>

disable UART AT command mode. If set to 0, the script will not
send `AT` commands and will not expect `OK` responses. Default is 1 (enabled).

=item B<bt_listen_addr=BDADDR_ANY|BDADDR_LOCAL|XX:XX:XX:XX:XX:XX>

local Bluetooth address to listen on (default: BDADDR_ANY).

=item B<security_level=none|low|medium|high|fips>

BLE security level for this specific connection. Overrides global setting.

=item B<io_capability=display-only|display-yesno|keyboard-only|no-input-output|keyboard-display>

IO capability for BLE pairing for this specific connection.

=item B<pin=NNNN>

4-6 digit PIN for BLE pairing authentication for this specific connection.

=back

=back


=head1 OPTIONS

=head2 Command Line Options

=over 4

=item B<--help>

Show help

=item B<--man>

Show full manual

=item B<--raw>, B<-r>

Enable raw mode - disables colored output, UTF-8 formatting, and fancy prompts.
Also sets log level to NONE unless explicitly configured. Useful for scripting
or when piping output.

=back

=head2 COMMANDS

The following commands can be entered at the prompt:

=over 4

=item B</connect XX:XX:XX:XX:XX:XX[,option=value]>

Connect to a new BLE device by address, optionally key=value pairs for config

Example:

    /connect 12:34:56:78:9A:BC
    /connect 12:34:56:78:9A:BC,uart_at=0


=item B</disconnect [XX:XX:XX:XX:XX:XX]>

Disconnect all BLE connections, or only the specified BLE device if a Bluetooth
address is given.

Example:

    /disconnect
    /disconnect 12:34:56:78:9A:BC

=item B</script E<lt>fileE<gt>>

Execute commands from the specified file, one per line, as if entered at the
prompt. Blank lines and lines starting with '#' are ignored.

Example:

    /script mycommands.txt

=item B</exit>, B</quit>

Exit the program.

=item B</debug on|off>

Enable or disable debug logging (sets loglevel to DEBUG or INFO).

=item B</logging on|off>

Enable or disable info logging (sets loglevel to INFO or NONE).

=item B</loglevel E<lt>none|info|warn|error|debugE<gt>>

Set the log level explicitly.

=item B</help>

Show this help message (list of / commands).

=item B</usage>

Show the usage.

=item B</man>

Show the manpage.

=item B</switch E<lt>XX:XX:XX:XX:XX:XXE<gt>>

Switch the active BLE device for terminal input/output to the specified
connected device.

Example:

    /switch 12:34:56:78:9A:BC

If no address is given, lists all currently connected devices.

=item B</primary [start_handle] [end_handle]>

Discover and display primary services on the current BLE device using GATT
service discovery. Output format matches gatttool --primary for compatibility.

Handle range is optional - if not provided, discovers all services (0x0001 to 0xFFFF).

Examples:

    /primary                   # Discover all primary services
    /primary 0x0001 0x00FF     # Discover services in specific handle range

Output format:

    attr handle: 0x0001, end grp handle: 0x0005 uuid: 0x1800
    attr handle: 0x0014, end grp handle: 0x001c uuid: 6e400001-b5a3-f393-e0a9-e50e24dcca9e

=item B</char-desc [start_handle] [end_handle]>

Discover and display characteristic descriptors on the current BLE device using
GATT descriptor discovery (Find Information Request). Output format matches
gatttool --char-desc for compatibility.

Handle range is optional - if not provided, discovers all descriptors (0x0001
to 0xFFFF).

Examples:

    /char-desc                 # Discover all characteristic descriptors
    /char-desc 0x0001 0x00FF   # Discover descriptors in specific handle range
    /char-desc 0x0015 0x0020   # Discover descriptors for specific characteristic

Output format:

    handle: 0x0002, uuid: 2800
    handle: 0x0003, uuid: 2803
    handle: 0x0005, uuid: 2902
    handle: 0x0019, uuid: 00002902-0000-1000-8000-00805f9b34fb

=item B</info>

Read and display the device name from the current BLE connection using the
Device Name characteristic (UUID: 00002a00-0000-1000-8000-00805f9b34fb).
This provides basic device information.

Example:

    /info

Output format:

    Device Name: ESP32-BLE-Device

=item B</security>

Display current security settings for the active connection or global defaults.
Shows security level, IO capability, and PIN status.

Example:

    /security

=item B</pair [XX:XX:XX:XX:XX:XX] [PIN]>

Initiate BLE pairing with the specified device or current connection.
Optionally provide a PIN for authentication.

Example:

    /pair
    /pair 12:34:56:78:9A:BC
    /pair 12:34:56:78:9A:BC 123456

=item B</unpair [XX:XX:XX:XX:XX:XX]>

Remove BLE pairing with the specified device or current connection.

Example:

    /unpair
    /unpair 12:34:56:78:9A:BC

=item B</script E<lt>fileE<gt>>

Execute commands from the specified file, one per line, as if entered at the
prompt. Blank lines and lines starting with '#' are ignored.
Example:

    /script mycommands.txt

This script understands perl within {...} blocks, allowing for dynamic command
generation. For example:

    /script commands.txt

Where commands.txt contains:

    # This is a comment
    /connect 12:34:56:78:9A:BC
    {
        for my $i (1..5) {
            print "/send Message number $i\n";
        }
    }
    /disconnect 12:34:56:78:9A:BC

Extra arguments can be specified after --, and will be available in the
script via @ARGV.

You can combine this script at commandline with other commands
like so:

    ./ble_uart.pl XX:XX:XX:XX:XX:XX,security_level=low,addr_type=public \
        --script mycommands.txt -- arg1 arg2 arg3

=back


=head1 SECURITY FEATURES

=head2 BLE Security Profiles

This script supports different BLE security profiles:

=over 4

=item * B<none> - No security requirements (BT_SECURITY_SDP)

=item * B<low> - Basic security, no authentication required (BT_SECURITY_LOW) [default]

=item * B<medium> - Medium security with authentication and encryption (BT_SECURITY_MEDIUM)

=item * B<high> - High security with strong encryption (BT_SECURITY_HIGH)

=item * B<fips> - FIPS-approved cryptographic algorithms (BT_SECURITY_FIPS)

=back

=head2 IO Capabilities

BLE pairing supports different IO capabilities for authentication:

=over 4

=item * B<display-only> - Device can only display information to user

=item * B<display-yesno> - Device can display information and accept yes/no input

=item * B<keyboard-only> - Device has keyboard input capability only

=item * B<no-input-output> - No input or output capabilities [default]

=item * B<keyboard-display> - Device has both keyboard input and display output

=back

=head2 PIN Authentication

When using medium or higher security profiles, PIN authentication may be required.
PINs must be 4-6 digits long.

=head1 EXAMPLES

=head2 Basic Usage

Connect to a BLE device with default security:

    ./ble_uart.pl 12:34:56:78:9A:BC

=head2 High Security Connection

Connect with high security profile and PIN:

    ./ble_uart.pl --security high --pin 123456 12:34:56:78:9A:BC

Or using per-connection options:

    ./ble_uart.pl 12:34:56:78:9A:BC,security_level=high,pin=123456

=head2 Interactive Pairing

Connect and then initiate pairing interactively:

    ./ble_uart.pl 12:34:56:78:9A:BC
    /security
    /pair 123456

=head2 Multiple Devices with Different Security

    ./ble_uart.pl 12:34:56:78:9A:BC,security_level=low 34:56:78:9A:BC:DE,security_level=high,pin=654321

=head2 Environment Configuration

Set defaults via environment variables:

    export BLE_UART_SECURITY_PROFILE=medium
    export BLE_UART_IO_CAPABILITY=keyboard-display
    export BLE_UART_PIN=123456
    ./ble_uart.pl 12:34:56:78:9A:BC


=head1 ENVIRONMENT

The following environment variables affect the behavior of this script:

=over 4

=item B<BLE_UART_DIR>

Directory for history and config files (default: ~/.ble_uart). Note that the
defaulting is done via the HOME and LOGNAME environment variables.

=item B<BLE_UART_HISTORY_FILE>

History file location (default: ~/.ble_uart_history).

=item B<BLE_UART_RAW>

Enable raw mode (default: 0). If set to 1, disables colored output, UTF-8
formatting, and fancy prompts. Also sets log level to NONE unless explicitly
configured.

=item B<BLE_UART_LOGLEVEL>

Set the log level (default: info). Can be set to debug, info, error, or none.

=item B<BLE_UART_INTERACTIVE_COLOR>

Enable colored output (default: 1).

=item B<BLE_UART_INTERACTIVE_UTF8>

Enable UTF-8 output (default: 1).

=item B<BLE_UART_INTERACTIVE_MULTILINE>

Enable multiline input (default: 1).

=item B<BLE_UART_SECURITY_PROFILE>

Set default security profile (default: low). Can be none, low, medium, high, or fips.

=item B<BLE_UART_IO_CAPABILITY>

Set default IO capability (default: no-input-output). Can be display-only,
display-yesno, keyboard-only, no-input-output, or keyboard-display.

=item B<BLE_UART_PIN>

Set default PIN for BLE pairing (4-6 digits).

=item B<TERM>

Terminal type, used to determine if colors are supported, if not set, "vt220"
is assumed.

=item B<COLORTERM>

This is checked for color support, if set to "truecolor" or "24bit", it will
enable true color support. If not set, "truecolor" is assumed.

=item B<MANPAGER>

Used for displaying the manpage, see the manpage of "man". This is usually
"less". If not set, "less" is used.

=back

=head1 AUTHOR

CowboyTim

=head1 LICENSE

This software is released under the Unlicense. See L<https://unlicense.org> for
details.

=cut
