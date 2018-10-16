# -*- mode: ruby -*-
# vi: set ft=ruby :

# All Vagrant configuration is done below. The "2" in Vagrant.configure
# configures the configuration version (we support older styles for
# backwards compatibility). Please don't change it unless you know what
# you're doing.
Vagrant.configure("2") do |config|
  # The most common configuration options are documented and commented below.
  # For a complete reference, please see the online documentation at
  # https://docs.vagrantup.com.

  # Every Vagrant development environment requires a box. You can search for
  # boxes at https://vagrantcloud.com/search.
  config.vm.box = "ubuntu/artful64"

  # Disable automatic box update checking. If you disable this, then
  # boxes will only be checked for updates when the user runs
  # `vagrant box outdated`. This is not recommended.
  # config.vm.box_check_update = false

  # Provider-specific configuration so you can fine-tune various
  # backing providers for Vagrant. These expose provider-specific options.
  # Example for VirtualBox:
  #
  config.vm.provider "virtualbox" do |vb|
    # Display the VirtualBox GUI when booting the machine
    # vb.gui = true

    # Customize the amount of memory on the VM:
    vb.memory = "2048"
  end

  # Enable provisioning with a shell script. Additional provisioners such as
  # Puppet, Chef, Ansible, Salt, and Docker are also available. Please see the
  # documentation for more information about their specific syntax and use.
  config.vm.provision "shell", inline: <<-SHELL
    apt-get update -y
    apt-get install -y sudo bash-completion vim ntp htop build-essential g++-multilib cmake python-dev git gdb emacs

    sudo -u vagrant bash -c "\
        cd /vagrant ; \
        if [ -f soplex-3.1.1.tgz ] && ! [ -e '/home/vagrant/local/soplex-3.1.1' ] ; then \
            echo 'export DOWNWARD_SOPLEX_ROOT32=/home/vagrant/local/soplex-3.1.1/32-bit/' >> /home/vagrant/.bash_profile ; \
            echo 'export DOWNWARD_SOPLEX_ROOT64=/home/vagrant/local/soplex-3.1.1/64-bit/' >> /home/vagrant/.bash_profile ; \
            tar xf soplex-3.1.1.tgz ; \
            cd soplex-3.1.1/ ; \
            mkdir -p build64 ; \
            cd build64/ ; \
            cmake -DCMAKE_INSTALL_PREFIX='/home/vagrant/local/soplex-3.1.1/64-bit' -DZLIB=False -DGMP=False .. ; \
            make -j 4 ; \
            make install ; \
            mkdir -p build64 ; \
            cd ../ ; 
            mkdir -p build32 ; \
            cd build32/ ; \
            CXXFLAGS='-m32' LDFLAGS='-m32' cmake -DCMAKE_INSTALL_PREFIX='/home/vagrant/local/soplex-3.1.1/32-bit' -DZLIB=False -DGMP=False .. ; \
            make -j ; \
            make install ; \
        fi"

      SHELL

  config.ssh.forward_x11 = true
end
